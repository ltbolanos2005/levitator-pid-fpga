-- =====================================================================
-- lev_top.vhd
-- Nivel superior del levitador (parte 2: PID + Arduino + esclavo MODBUS).
--
--   Arduino (HC-SR04 + PWM) <--UART 9600--> arduino_link <--> pid_ctrl
--                                                  |  puerto 1      |
--                                               reg_bank  <---------+
--                                                  |  puerto 2
--   ESP32 (maestro) <--RS485 9600--> modbus_rtu_slave <--> mb_regmap
--
-- Pines:  clk (50 MHz), rst_n_in (boton, activo en bajo),
--         ard_rx (viene del TX del Arduino, pasar por divisor 5V->3.3V),
--         ard_tx (va al RX del Arduino),
--         mb_rx  (<- TXD/RO del modulo RS485 hacia el ESP32),
--         mb_tx  (-> RXD/DI del modulo RS485),
--         mb_de  (DE/RE del transceptor; sin uso con modulos de flujo
--                 automatico, se deja por si se cambia de modulo),
--         leds(5:0) (activos en alto).
-- =====================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.lev_pkg.all;

entity lev_top is
    generic (
        CLK_FREQ_HZ : integer := 50_000_000;
        BAUD_RATE   : integer := 9600;              -- enlace con el Arduino
        MB_BAUD     : integer := 9600;              -- MODBUS RTU con el ESP32
        MB_ID       : integer range 1 to 247 := 1;  -- direccion del esclavo
        PWM_MIN     : integer := DEF_PWM_MIN;
        -- Valores de arranque en la tarjeta: sintonia hecha en la planta
        -- real el 2026-11-10 (BIAS 160, KP 2.0, KI 0.0156, KD 4.0, SP 230 mm).
        -- El banco de pruebas los reemplaza por los DEF_* de lev_pkg.
        INIT_SP     : integer := 230;
        INIT_KP     : integer := 8192;    -- 2.0    PWM/mm
        INIT_KI     : integer := 64;      -- 0.0156 PWM/(mm*muestra)
        INIT_KD     : integer := 16384;   -- 4.0    PWM/(mm/muestra)
        INIT_EN     : integer := 1;       -- arranca con el lazo activo
        INIT_BIAS   : integer := 160
    );
    port (
        clk      : in  std_logic;
        rst_n_in : in  std_logic;
        ard_rx   : in  std_logic;
        ard_tx   : out std_logic;
        mb_rx    : in  std_logic;
        mb_tx    : out std_logic;
        mb_de    : out std_logic;
        leds     : out std_logic_vector(5 downto 0)
    );
end entity lev_top;

architecture rtl of lev_top is
    constant HOLD_CLKS : integer := (CLK_FREQ_HZ / 1000) * 500;   -- 0.5 s

    signal rst_sr     : std_logic_vector(1 downto 0) := "00";
    signal rst_n      : std_logic;

    signal regs       : reg_array_t;
    signal wr_en      : std_logic;
    signal wr_addr    : integer range 0 to NUM_REGS - 1;
    signal wr_data    : std_logic_vector(15 downto 0);
    signal frame_bad  : std_logic;

    signal meas_valid : std_logic;
    signal meas_pv    : std_logic_vector(15 downto 0);
    signal pid_done   : std_logic;
    signal pid_mv     : std_logic_vector(7 downto 0);
    signal pid_err    : std_logic_vector(15 downto 0);
    signal pid_pv     : std_logic_vector(15 downto 0);
    -- (VHDL no distingue mayusculas: no usar nombres como ST_SENSOR, ya definidos en lev_pkg)
    signal f_sensor, f_timeout, f_sat_hi, f_sat_lo : std_logic;
    signal mv_word, status_word : word_t;

    signal status     : std_logic_vector(7 downto 0) := (others => '0');
    signal frame_hold : integer range 0 to HOLD_CLKS := 0;
    signal blink      : std_logic := '0';

    -- MODBUS
    signal mb_regs    : mb_array_t;
    signal mbs_wr_en  : std_logic;
    signal mbs_wr_addr: integer range 0 to MB_NUM_REGS - 1;
    signal mbs_wr_data: word_t;
    signal wr2_en     : std_logic;
    signal wr2_addr   : integer range 0 to NUM_REGS - 1;
    signal wr2_data   : word_t;
    signal mb_ok, mb_bad, mb_exc, mb_done : std_logic;
    signal mb_blink   : std_logic := '0';
    signal mb_err_hold: integer range 0 to HOLD_CLKS := 0;
begin

    -- Reset: se activa de inmediato, se libera sincronizado con clk
    process (clk, rst_n_in)
    begin
        if rst_n_in = '0' then
            rst_sr <= "00";
        elsif rising_edge(clk) then
            rst_sr <= rst_sr(0) & '1';
        end if;
    end process;
    rst_n <= rst_sr(1);

    u_link : entity work.arduino_link
        generic map (CLK_FREQ_HZ => CLK_FREQ_HZ, BAUD_RATE => BAUD_RATE)
        port map (clk => clk, rst_n => rst_n, rxd => ard_rx, txd => ard_tx,
                  meas_valid => meas_valid, meas_pv => meas_pv,
                  pid_done => pid_done, mv => pid_mv, status => status,
                  regs => regs, wr_en => wr_en, wr_addr => wr_addr, wr_data => wr_data,
                  frame_bad => frame_bad);

    u_pid : entity work.pid_ctrl
        generic map (CLK_FREQ_HZ => CLK_FREQ_HZ, PWM_MIN => PWM_MIN)
        port map (clk => clk, rst_n => rst_n,
                  sample_valid => meas_valid, pv => meas_pv,
                  sp => regs(REG_SP), kp => regs(REG_KP), ki => regs(REG_KI),
                  kd => regs(REG_KD), bias => regs(REG_BIAS),
                  enable => regs(REG_ENABLE)(0),
                  mv => pid_mv, err => pid_err, pv_reg => pid_pv,
                  st_sensor => f_sensor, st_timeout => f_timeout,
                  st_sat_high => f_sat_hi, st_sat_low => f_sat_lo,
                  done => pid_done);

    u_regs : entity work.reg_bank
        generic map (INIT_SP => INIT_SP, INIT_KP => INIT_KP, INIT_KI => INIT_KI,
                     INIT_KD => INIT_KD, INIT_EN => INIT_EN, INIT_BIAS => INIT_BIAS)
        port map (clk => clk, rst_n => rst_n,
                  wr1_en => wr_en, wr1_addr => wr_addr, wr1_data => wr_data,
                  wr2_en => wr2_en, wr2_addr => wr2_addr, wr2_data => wr2_data,
                  pv_in => pid_pv, mv_in => mv_word,
                  err_in => pid_err, status_in => status_word,
                  regs => regs);

    -- -----------------------------------------------------------------
    -- Esclavo MODBUS RTU (ESP32) y traductor de mapa/unidades
    -- -----------------------------------------------------------------
    u_mb : entity work.modbus_rtu_slave
        generic map (CLK_FREQ_HZ => CLK_FREQ_HZ, BAUD_RATE => MB_BAUD, SLAVE_ID => MB_ID)
        port map (clk => clk, rst_n => rst_n, rxd => mb_rx, txd => mb_tx, de => mb_de,
                  regs_in => mb_regs,
                  wr_en => mbs_wr_en, wr_addr => mbs_wr_addr, wr_data => mbs_wr_data,
                  frame_ok => mb_ok, frame_bad => mb_bad, exc_sent => mb_exc,
                  tx_done => mb_done);

    u_mbmap : entity work.mb_regmap
        port map (clk => clk, rst_n => rst_n,
                  mb_wr_en => mbs_wr_en, mb_wr_addr => mbs_wr_addr, mb_wr_data => mbs_wr_data,
                  mb_regs => mb_regs,
                  regs => regs, wr_en => wr2_en, wr_addr => wr2_addr, wr_data => wr2_data);

    mv_word     <= x"00" & pid_mv;
    status_word <= x"00" & status;

    -- Banderas de estado
    status(ST_ENABLED)  <= regs(REG_ENABLE)(0);
    status(ST_SENSOR)   <= f_sensor;
    status(ST_TIMEOUT)  <= f_timeout;
    status(ST_SAT_HIGH) <= f_sat_hi;
    status(ST_SAT_LOW)  <= f_sat_lo;
    status(ST_FRAME)    <= '1' when frame_hold /= 0 else '0';
    status(7 downto 6)  <= "00";

    -- Retencion de la bandera de error de trama y parpadeo de actividad
    process (clk, rst_n)
    begin
        if rst_n = '0' then
            frame_hold <= 0;
            blink      <= '0';
        elsif rising_edge(clk) then
            if frame_bad = '1' then
                frame_hold <= HOLD_CLKS;
            elsif frame_hold /= 0 then
                frame_hold <= frame_hold - 1;
            end if;
            if pid_done = '1' then
                blink <= not blink;
            end if;
        end if;
    end process;

    -- LEDs del enlace MODBUS
    process (clk, rst_n)
    begin
        if rst_n = '0' then
            mb_blink    <= '0';
            mb_err_hold <= 0;
        elsif rising_edge(clk) then
            if mb_ok = '1' then
                mb_blink <= not mb_blink;
            end if;
            if mb_bad = '1' or mb_exc = '1' then
                mb_err_hold <= HOLD_CLKS;
            elsif mb_err_hold /= 0 then
                mb_err_hold <= mb_err_hold - 1;
            end if;
        end if;
    end process;

    leds(0) <= regs(REG_ENABLE)(0);     -- lazo habilitado
    leds(1) <= f_sensor;               -- sensor sin eco
    leds(2) <= f_timeout;              -- Arduino sin comunicar
    leds(3) <= blink;                   -- parpadea con cada muestra
    leds(4) <= mb_blink;                -- cambia con cada trama MODBUS valida para este esclavo
    leds(5) <= '1' when mb_err_hold /= 0 else '0';   -- 0.5 s tras CRC malo o excepcion

end architecture rtl;
