-- =====================================================================
-- modbus_rtu_slave.vhd   (version 3)
-- Esclavo MODBUS RTU para el levitador (lado FPGA del enlace con el ESP32)
--
--   * UART 9600 8N1 (reutiliza uart_rx / uart_tx)
--   * Fin de trama: silencio de 3.5 caracteres (3.5 x 11 bits = 4.0 ms a 9600)
--   * Direccion propia (SLAVE_ID, por defecto 1) y difusion (direccion 0,
--     solo escrituras y sin respuesta, como indica la norma)
--   * CRC-16 MODBUS (0xA001, inicial 0xFFFF), calculado byte a byte al
--     recibir (sobre toda la trama, incluido el CRC, debe dar 0) y al enviar
--   * Funciones:
--       0x03 Read Holding Registers    (1..125 registros)
--       0x06 Write Single Register
--       0x10 Write Multiple Registers  (1..MB_NUM_REGS registros)
--   * Excepciones (en el orden que pide la norma):
--       0x01 funcion ilegal
--       0x03 dato ilegal   (longitud, cantidad, byte count o valor fuera de rango)
--       0x02 direccion ilegal (fuera del mapa o registro de solo lectura)
--   * 0x10 es atomica: se validan TODOS los registros antes de escribir
--     el primero; si alguno falla no se escribe ninguno.
--
-- Las escrituras salen por (wr_en, wr_addr, wr_data), un registro por
-- pulso, en direcciones del mapa MODBUS (MB_* de lev_pkg).  La lectura
-- toma una copia coherente de regs_in al decodificar la trama.
-- =====================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.lev_pkg.all;

entity modbus_rtu_slave is
    generic (
        CLK_FREQ_HZ : integer := 50_000_000;
        BAUD_RATE   : integer := 9600;
        SLAVE_ID    : integer range 1 to 247 := 1
    );
    port (
        clk       : in  std_logic;
        rst_n     : in  std_logic;
        rxd       : in  std_logic;
        txd       : out std_logic;
        de        : out std_logic;     -- '1' mientras se transmite (DE/RE del RS485)
        regs_in   : in  mb_array_t;    -- valores a leer (mapa MODBUS)
        wr_en     : out std_logic;     -- pulso: escribir wr_data en wr_addr
        wr_addr   : out integer range 0 to MB_NUM_REGS - 1;
        wr_data   : out word_t;
        frame_ok  : out std_logic;     -- pulso: trama valida para este esclavo
        frame_bad : out std_logic;     -- pulso: trama con CRC / bit de parada malo
        exc_sent  : out std_logic;     -- pulso: se respondio con excepcion
        tx_done   : out std_logic      -- pulso: respuesta enviada
    );
end entity modbus_rtu_slave;

architecture rtl of modbus_rtu_slave is
    constant CLKS_PER_BIT : integer := CLK_FREQ_HZ / BAUD_RATE;
    constant GAP_CLKS     : integer := (CLKS_PER_BIT * 11 * 7) / 2;   -- 3.5 caracteres
    constant RX_MAX       : integer := 9 + 2 * MB_NUM_REGS;           -- trama 0x10 mas larga
    constant MY_ADDR      : std_logic_vector(7 downto 0) := std_logic_vector(to_unsigned(SLAVE_ID, 8));

    type rx_arr_t is array (0 to RX_MAX - 1) of std_logic_vector(7 downto 0);

    signal rx_data  : std_logic_vector(7 downto 0);
    signal rx_valid : std_logic;
    signal rx_ferr  : std_logic;
    signal tx_start : std_logic := '0';
    signal tx_data  : std_logic_vector(7 downto 0) := (others => '0');
    signal tx_busy  : std_logic;

    type state_t is (S_IDLE, S_RECV, S_CHECK, S_ADDR, S_DECODE, S_SCAN, S_WRITE,
                     S_PREP, S_GO, S_WAITB, S_WAITD);
    signal state    : state_t := S_IDLE;

    -- Recepcion
    signal rbuf     : rx_arr_t := (others => (others => '0'));
    signal rlen     : integer range 0 to RX_MAX := 0;
    signal rovf     : std_logic := '0';
    signal bad_char : std_logic := '0';
    signal crc_rx   : word_t := x"FFFF";
    -- Tolerancia a ruido del bus (modulos RS485 de flujo automatico):
    -- crc_b se calcula desde el SEGUNDO byte (por si se cuela un byte basura
    -- antes de la trama); good_a / good_b guardan la ultima longitud en la
    -- que el CRC dio 0 (por si se cuela basura despues de la trama).
    signal crc_b    : word_t := x"FFFF";
    signal good_a   : integer range 0 to RX_MAX := 0;
    signal good_b   : integer range 0 to RX_MAX := 0;
    signal gap_cnt  : integer range 0 to GAP_CLKS := 0;
    signal gap_done : std_logic := '0';
    signal bcast    : std_logic := '0';

    -- Ejecucion
    signal fcode    : std_logic_vector(7 downto 0) := x"00";
    signal start_a  : integer range 0 to 65535 := 0;
    signal qty      : integer range 0 to 65535 := 0;
    signal idx      : integer range 0 to MB_NUM_REGS := 0;
    signal bad_addr : std_logic := '0';
    signal bad_val  : std_logic := '0';

    -- Respuesta
    type kind_t is (K_READ, K_ECHO, K_EXC);
    signal kind     : kind_t := K_EXC;
    signal exc_code : std_logic_vector(7 downto 0) := x"00";
    signal snap     : mb_array_t := (others => (others => '0'));
    signal dlen     : integer range 0 to 3 + 2 * MB_NUM_REGS := 0;   -- bytes sin CRC
    signal tpos     : integer range 0 to 3 + 2 * MB_NUM_REGS + 2 := 0;
    signal crc_tx   : word_t := x"FFFF";
    signal cur_byte : std_logic_vector(7 downto 0) := x"00";
    signal cur_crc  : std_logic := '0';
    signal de_i     : std_logic := '0';
begin
    de <= de_i;

    u_rx : entity work.uart_rx
        generic map (CLKS_PER_BIT => CLKS_PER_BIT)
        port map (clk => clk, rst_n => rst_n, rxd => rxd,
                  data => rx_data, valid => rx_valid, frame_err => rx_ferr);

    u_tx : entity work.uart_tx
        generic map (CLKS_PER_BIT => CLKS_PER_BIT)
        port map (clk => clk, rst_n => rst_n, start => tx_start, data => tx_data,
                  txd => txd, busy => tx_busy);

    -- -----------------------------------------------------------------
    -- Detector de fin de trama: 3.5 caracteres sin recibir nada
    -- -----------------------------------------------------------------
    process (clk, rst_n)
    begin
        if rst_n = '0' then
            gap_cnt  <= 0;
            gap_done <= '0';
        elsif rising_edge(clk) then
            gap_done <= '0';
            if rx_valid = '1' or rx_ferr = '1' or state /= S_RECV then
                gap_cnt <= 0;
            elsif gap_cnt = GAP_CLKS then
                gap_done <= '1';
            else
                gap_cnt <= gap_cnt + 1;
            end if;
        end if;
    end process;

    -- -----------------------------------------------------------------
    -- Maquina de estados principal
    -- -----------------------------------------------------------------
    process (clk, rst_n)
        variable addr_v, qty_v : integer range 0 to 65535;
        variable a             : integer range 0 to 65535;
        variable w             : word_t;
        variable b             : std_logic_vector(7 downto 0);
        variable k             : integer range 0 to 255;

        -- Dato del registro i de una trama 0x10 (bytes 7+2i y 8+2i)
        impure function wdata(i : integer) return word_t is
        begin
            return rbuf(7 + 2 * i) & rbuf(8 + 2 * i);
        end function;
    begin
        if rst_n = '0' then
            state     <= S_IDLE;
            rlen      <= 0;
            rovf      <= '0';
            bad_char  <= '0';
            crc_rx    <= x"FFFF";
            bcast     <= '0';
            de_i      <= '0';
            tx_start  <= '0';
            tx_data   <= (others => '0');
            wr_en     <= '0';
            wr_addr   <= 0;
            wr_data   <= (others => '0');
            frame_ok  <= '0';
            frame_bad <= '0';
            exc_sent  <= '0';
            tx_done   <= '0';
            idx       <= 0;
            tpos      <= 0;
        elsif rising_edge(clk) then
            tx_start  <= '0';
            wr_en     <= '0';
            frame_ok  <= '0';
            frame_bad <= '0';
            exc_sent  <= '0';
            tx_done   <= '0';

            case state is
                -- ---------------------------------------------- recepcion
                when S_IDLE =>
                    de_i <= '0';
                    if rx_valid = '1' then
                        rbuf(0)  <= rx_data;
                        rlen     <= 1;
                        rovf     <= '0';
                        bad_char <= '0';
                        crc_rx   <= crc16_update(x"FFFF", rx_data);
                        crc_b    <= x"FFFF";
                        good_a   <= 0;
                        good_b   <= 0;
                        state    <= S_RECV;
                    end if;
                    -- un caracter con error de parada en reposo es ruido del bus:
                    -- se ignora (no abre una trama)

                when S_RECV =>
                    if rx_valid = '1' then
                        if rlen < RX_MAX then
                            rbuf(rlen) <= rx_data;
                            rlen <= rlen + 1;
                            -- CRC desde el byte 0 y desde el byte 1. "crc /= 0" evita que
                            -- ceros de basura al final (CRC sigue en 0) alarguen la trama
                            if crc16_update(crc_rx, rx_data) = x"0000" and crc_rx /= x"0000" and rlen + 1 >= 4 and bad_char = '0' then
                                good_a <= rlen + 1;
                            end if;
                            if rlen >= 1 then
                                if crc16_update(crc_b, rx_data) = x"0000" and crc_b /= x"0000" and rlen >= 4 and bad_char = '0' then
                                    good_b <= rlen;      -- longitud sin el primer byte
                                end if;
                                crc_b <= crc16_update(crc_b, rx_data);
                            end if;
                        else
                            rovf <= '1';                 -- trama mas larga que el buffer
                        end if;
                        crc_rx <= crc16_update(crc_rx, rx_data);
                    elsif rx_ferr = '1' then
                        -- error de parada: solo invalida lo que venga despues si aun
                        -- no hay una trama con CRC correcto (basura al final se ignora)
                        if good_a = 0 and good_b = 0 then
                            bad_char <= '1';
                        end if;
                    elsif gap_done = '1' then
                        state <= S_CHECK;
                    end if;

                -- CRC, caracteres y direccion
                when S_CHECK =>
                    if good_a >= 4 then
                        rlen <= good_a;                  -- se descarta basura al final
                        state <= S_ADDR;
                    elsif good_b >= 4 then
                        -- basura antes de la trama: correr el buffer un byte
                        for i in 0 to RX_MAX - 2 loop
                            rbuf(i) <= rbuf(i + 1);
                        end loop;
                        rlen  <= good_b;
                        state <= S_ADDR;
                    else
                        frame_bad <= '1';                -- trama corrupta: se descarta
                        state     <= S_IDLE;
                    end if;

                when S_ADDR =>
                    if rbuf(0) = MY_ADDR or rbuf(0) = x"00" then
                        if rbuf(0) = x"00" then bcast <= '1'; else bcast <= '0'; end if;
                        frame_ok <= '1';
                        state    <= S_DECODE;
                    else
                        state <= S_IDLE;                 -- es para otro esclavo
                    end if;

                -- ---------------------------------------------- decodificacion
                when S_DECODE =>
                    fcode   <= rbuf(1);
                    addr_v  := to_integer(unsigned(std_logic_vector'(rbuf(2) & rbuf(3))));
                    qty_v   := to_integer(unsigned(std_logic_vector'(rbuf(4) & rbuf(5))));
                    start_a <= addr_v;
                    qty     <= qty_v;
                    kind    <= K_EXC;
                    dlen    <= 3;
                    tpos    <= 0;
                    crc_tx  <= x"FFFF";
                    idx     <= 0;
                    bad_addr <= '0';
                    bad_val  <= '0';
                    state   <= S_PREP;                   -- por defecto: responder

                    if rbuf(1) = x"03" then
                        if rovf = '1' or rlen /= 8 or qty_v < 1 or qty_v > 125 then
                            exc_code <= x"03";
                        elsif addr_v + qty_v > MB_NUM_REGS then
                            exc_code <= x"02";
                        else
                            kind <= K_READ;
                            snap <= regs_in;             -- copia coherente
                            dlen <= 3 + 2 * qty_v;
                        end if;
                        if bcast = '1' then state <= S_IDLE; end if;   -- lectura difundida: se ignora

                    elsif rbuf(1) = x"06" then
                        -- en 0x06 los bytes 4-5 son el valor, no una cantidad
                        if rovf = '1' or rlen /= 8 then
                            exc_code <= x"03";
                        elsif addr_v >= MB_NUM_REGS or not mb_writable(addr_v) then
                            exc_code <= x"02";
                        elsif not mb_value_ok(addr_v, rbuf(4) & rbuf(5)) then
                            exc_code <= x"03";
                        else
                            wr_addr <= addr_v;
                            wr_data <= rbuf(4) & rbuf(5);
                            wr_en   <= '1';
                            kind    <= K_ECHO;           -- respuesta = eco de 6 bytes
                            dlen    <= 6;
                        end if;

                    elsif rbuf(1) = x"10" then
                        if rovf = '1' or rlen < 11 or qty_v < 1 or qty_v > 123 or
                           to_integer(unsigned(rbuf(6))) /= 2 * qty_v or rlen /= 9 + 2 * qty_v then
                            exc_code <= x"03";
                        elsif addr_v + qty_v > MB_NUM_REGS then
                            exc_code <= x"02";
                        else
                            state <= S_SCAN;             -- validar cada registro
                        end if;

                    else
                        exc_code <= x"01";
                    end if;

                -- 0x10: revisa todos los registros antes de escribir
                when S_SCAN =>
                    if idx < qty then
                        a := start_a + idx;
                        if not mb_writable(a) then
                            bad_addr <= '1';
                        elsif not mb_value_ok(a, wdata(idx)) then
                            bad_val <= '1';
                        end if;
                        idx <= idx + 1;
                    else
                        idx <= 0;
                        if bad_addr = '1' then
                            exc_code <= x"02";
                            state    <= S_PREP;
                        elsif bad_val = '1' then
                            exc_code <= x"03";
                            state    <= S_PREP;
                        else
                            state <= S_WRITE;
                        end if;
                    end if;

                -- 0x10: una escritura por ciclo
                when S_WRITE =>
                    if idx < qty then
                        wr_addr <= start_a + idx;
                        wr_data <= wdata(idx);
                        wr_en   <= '1';
                        idx     <= idx + 1;
                    else
                        kind  <= K_ECHO;                 -- respuesta: dir, fc, inicio, cantidad
                        dlen  <= 6;
                        state <= S_PREP;
                    end if;

                -- ---------------------------------------------- respuesta
                when S_PREP =>
                    if bcast = '1' then
                        state <= S_IDLE;                 -- difusion: no se responde
                    elsif tpos < dlen then
                        case kind is
                            when K_READ =>
                                if tpos = 0 then b := MY_ADDR;
                                elsif tpos = 1 then b := x"03";
                                elsif tpos = 2 then b := std_logic_vector(to_unsigned(2 * qty, 8));
                                else
                                    k := (tpos - 3) / 2;
                                    if start_a + k < MB_NUM_REGS then w := snap(start_a + k);
                                    else w := x"0000"; end if;
                                    if ((tpos - 3) mod 2) = 0 then b := w(15 downto 8);
                                    else b := w(7 downto 0); end if;
                                end if;
                            when K_ECHO =>
                                b := rbuf(tpos);
                            when K_EXC =>
                                if tpos = 0 then b := MY_ADDR;
                                elsif tpos = 1 then b := fcode or x"80";
                                else b := exc_code; end if;
                        end case;
                        if tpos = 0 then de_i <= '1'; end if;
                        cur_byte <= b;
                        cur_crc  <= '0';
                        state    <= S_GO;
                    elsif tpos = dlen then
                        cur_byte <= crc_tx(7 downto 0);  -- CRC: byte bajo primero
                        cur_crc  <= '1';
                        state    <= S_GO;
                    elsif tpos = dlen + 1 then
                        cur_byte <= crc_tx(15 downto 8);
                        cur_crc  <= '1';
                        state    <= S_GO;
                    else
                        de_i    <= '0';                  -- libera el bus
                        tx_done <= '1';
                        if kind = K_EXC then exc_sent <= '1'; end if;
                        state   <= S_IDLE;
                    end if;

                when S_GO =>
                    tx_data  <= cur_byte;
                    tx_start <= '1';
                    if cur_crc = '0' then
                        crc_tx <= crc16_update(crc_tx, cur_byte);
                    end if;
                    state <= S_WAITB;

                when S_WAITB =>
                    if tx_busy = '1' then state <= S_WAITD; end if;

                when S_WAITD =>
                    if tx_busy = '0' then
                        tpos  <= tpos + 1;
                        state <= S_PREP;
                    end if;
            end case;
        end if;
    end process;

end architecture rtl;
