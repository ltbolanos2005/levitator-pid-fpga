-- =====================================================================
-- pid_ctrl.vhd   (version 2)
-- Controlador PID discreto en punto fijo para el levitador neumatico.
--
--   e[k] = PV[k] - SP        (sensor arriba: mas distancia = pelota mas
--                             abajo = hay que soplar mas)
--   P    = KP * e[k]
--   I    = KI * sum(e)       (anti-windup por "clamping")
--   D    = KD * dPV_filtrada (derivada sobre la MEDICION + filtro IIR)
--   MV   = BIAS + (P + I + D) / 4096
--   MV   saturado a [PWM_MIN .. PWM_MAX] mientras el lazo esta activo
--        (0 solo si el lazo esta detenido, hay falla de sensor o timeout)
--
-- Ganancias en formato x/4096 (GAIN_SHIFT en lev_pkg; KP = 4096 -> 1.0 PWM por mm).
-- Se ejecuta una vez por cada medicion (pulso sample_valid).
-- Los tiempos de muestreo supuestos son los del Arduino (50 ms).
-- =====================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.lev_pkg.all;

entity pid_ctrl is
    generic (
        CLK_FREQ_HZ  : integer := 50_000_000;
        WDOG_MS      : integer := 250;          -- sin mediciones -> falla
        PWM_MIN      : integer := DEF_PWM_MIN;  -- el ventilador no gira por debajo
        PWM_MAX      : integer := 255;
        D_FILT_SHIFT : integer := 1             -- filtro derivada: alfa = 1/2^N
    );
    port (
        clk          : in  std_logic;
        rst_n        : in  std_logic;
        sample_valid : in  std_logic;
        pv           : in  std_logic_vector(15 downto 0);  -- mm (x"FFFF" = falla)
        sp           : in  std_logic_vector(15 downto 0);
        kp           : in  std_logic_vector(15 downto 0);
        ki           : in  std_logic_vector(15 downto 0);
        kd           : in  std_logic_vector(15 downto 0);
        bias         : in  std_logic_vector(15 downto 0);
        enable       : in  std_logic;
        mv           : out std_logic_vector(7 downto 0);
        err          : out std_logic_vector(15 downto 0);
        pv_reg       : out std_logic_vector(15 downto 0);
        st_sensor    : out std_logic;
        st_timeout   : out std_logic;
        st_sat_high  : out std_logic;
        st_sat_low   : out std_logic;
        done         : out std_logic           -- pulso: mv listo
    );
end entity pid_ctrl;

architecture rtl of pid_ctrl is
    constant DFRAC      : integer := 4;                       -- derivada con 4 bits fraccionarios
    constant E_LIM      : integer := 8191;                    -- |e| y |dPV| saturados (mm)
    constant I_TERM_LIM : integer := PWM_MAX * (2 ** GAIN_SHIFT);  -- el termino I aporta <= PWM_MAX
    constant IACC_LIM   : integer := 2 ** 20;
    constant WDOG_CLKS  : integer := (CLK_FREQ_HZ / 1000) * WDOG_MS;

    type state_t is (S_IDLE, S_ERR, S_MUL, S_SUM, S_OUT);
    signal state    : state_t := S_IDLE;

    signal pv_l     : std_logic_vector(15 downto 0) := (others => '0');
    signal fault_l  : std_logic := '0';
    signal e        : signed(15 downto 0) := (others => '0');   -- error saturado
    signal d_f      : signed(19 downto 0) := (others => '0');   -- derivada filtrada (x16)
    signal pv_prev  : signed(17 downto 0) := (others => '0');
    signal first    : std_logic := '1';
    signal iacc     : signed(21 downto 0) := (others => '0');
    signal iacc_new : signed(21 downto 0) := (others => '0');
    signal p_prod   : signed(32 downto 0) := (others => '0');   -- 17 x 16
    signal d_prod   : signed(36 downto 0) := (others => '0');   -- 17 x 20
    signal i_prod   : signed(38 downto 0) := (others => '0');   -- 17 x 22
    signal sat_hi   : std_logic := '0';
    signal sat_lo   : std_logic := '0';
    signal timeout_i: std_logic := '0';
    signal wd_cnt   : integer range 0 to WDOG_CLKS := 0;

    signal mv_i     : std_logic_vector(7 downto 0) := (others => '0');
    signal err_i    : std_logic_vector(15 downto 0) := (others => '0');
    signal pv_out   : std_logic_vector(15 downto 0) := (others => '0');
    signal sensor_i : std_logic := '0';
begin
    mv          <= mv_i;
    err         <= err_i;
    pv_reg      <= pv_out;
    st_sensor   <= sensor_i;
    st_timeout  <= timeout_i;
    st_sat_high <= sat_hi;
    st_sat_low  <= sat_lo;

    process (clk, rst_n)
        variable pv_s, sp_s : signed(17 downto 0);
        variable e_full     : signed(17 downto 0);
        variable d_full     : signed(17 downto 0);
        variable diff       : signed(21 downto 0);
        variable tmp        : signed(21 downto 0);
        variable i_cl       : signed(38 downto 0);
        variable sum, u     : signed(41 downto 0);
        variable stop_i     : boolean;
    begin
        if rst_n = '0' then
            state     <= S_IDLE;
            first     <= '1';
            iacc      <= (others => '0');
            iacc_new  <= (others => '0');
            d_f       <= (others => '0');
            e         <= (others => '0');
            pv_prev   <= (others => '0');
            fault_l   <= '0';
            mv_i      <= (others => '0');
            err_i     <= (others => '0');
            pv_out    <= (others => '0');
            sensor_i  <= '0';
            sat_hi    <= '0';
            sat_lo    <= '0';
            timeout_i <= '0';
            wd_cnt    <= 0;
            done      <= '0';
        elsif rising_edge(clk) then
            done <= '0';

            -- Vigilancia: si no llegan mediciones, se declara falla
            if sample_valid = '1' then
                wd_cnt    <= 0;
                timeout_i <= '0';
            elsif wd_cnt = WDOG_CLKS then
                if timeout_i = '0' then
                    timeout_i <= '1';
                    mv_i      <= (others => '0');
                    iacc      <= (others => '0');
                    first     <= '1';
                    sat_hi    <= '0';
                    sat_lo    <= '0';
                end if;
            else
                wd_cnt <= wd_cnt + 1;
            end if;

            case state is
                when S_IDLE =>
                    if sample_valid = '1' then
                        pv_l <= pv;
                        if pv = PV_FAULT then fault_l <= '1'; else fault_l <= '0'; end if;
                        state <= S_ERR;
                    end if;

                -- Error (saturado) y derivada filtrada
                when S_ERR =>
                    pv_s   := signed(resize(unsigned(pv_l), 18));
                    sp_s   := signed(resize(unsigned(sp),   18));
                    e_full := pv_s - sp_s;
                    if e_full > E_LIM then
                        e_full := to_signed(E_LIM, 18);
                    elsif e_full < -E_LIM then
                        e_full := to_signed(-E_LIM, 18);
                    end if;
                    e <= resize(e_full, 16);

                    if first = '1' then
                        d_f <= (others => '0');          -- sin patada derivativa
                    else
                        d_full := pv_s - pv_prev;
                        if d_full > E_LIM then
                            d_full := to_signed(E_LIM, 18);
                        elsif d_full < -E_LIM then
                            d_full := to_signed(-E_LIM, 18);
                        end if;
                        -- d_f += (16*dPV - d_f) / 2^D_FILT_SHIFT
                        diff := shift_left(resize(d_full, 22), DFRAC) - resize(d_f, 22);
                        d_f  <= d_f + resize(shift_right(diff, D_FILT_SHIFT), 20);
                    end if;
                    state <= S_MUL;

                -- Productos e integrador candidato
                when S_MUL =>
                    p_prod <= signed('0' & kp) * e;
                    d_prod <= signed('0' & kd) * d_f;
                    tmp := iacc + resize(e, 22);
                    if tmp > to_signed(IACC_LIM, 22)  then tmp := to_signed(IACC_LIM, 22);  end if;
                    if tmp < to_signed(-IACC_LIM, 22) then tmp := to_signed(-IACC_LIM, 22); end if;
                    if unsigned(ki) = 0 then
                        tmp := (others => '0');          -- KI = 0: sin acumulacion oculta
                    end if;
                    iacc_new <= tmp;
                    i_prod   <= signed('0' & ki) * tmp;
                    state    <= S_SUM;

                -- Suma, escalado, saturacion, anti-windup y registro de salida
                when S_SUM =>
                    i_cl := i_prod;
                    if i_cl > to_signed(I_TERM_LIM, 39)  then i_cl := to_signed(I_TERM_LIM, 39);  end if;
                    if i_cl < to_signed(-I_TERM_LIM, 39) then i_cl := to_signed(-I_TERM_LIM, 39); end if;

                    sum := resize(p_prod, 42) + resize(i_cl, 42) + shift_right(resize(d_prod, 42), DFRAC);
                    u   := shift_right(sum, GAIN_SHIFT) + resize(signed('0' & bias), 42);

                    err_i <= std_logic_vector(e);

                    if fault_l = '1' or enable = '0' then
                        -- Lazo detenido o sensor sin eco: ventilador apagado
                        mv_i   <= (others => '0');
                        iacc   <= (others => '0');
                        first  <= '1';
                        sat_hi <= '0';
                        sat_lo <= '0';
                        if fault_l = '1' then err_i <= (others => '0'); end if;
                    else
                        first   <= '0';
                        pv_prev <= signed(resize(unsigned(pv_l), 18));

                        if u > PWM_MAX then
                            mv_i <= std_logic_vector(to_unsigned(PWM_MAX, 8));
                            sat_hi <= '1'; sat_lo <= '0';
                        elsif u < PWM_MIN then
                            mv_i <= std_logic_vector(to_unsigned(PWM_MIN, 8));
                            sat_hi <= '0'; sat_lo <= '1';
                        else
                            mv_i <= std_logic_vector(u(7 downto 0));
                            sat_hi <= '0'; sat_lo <= '0';
                        end if;

                        -- Anti-windup (clamping): no se acumula si la salida
                        -- (o el termino I) ya esta saturada en el sentido del error
                        stop_i := (u > PWM_MAX and e > 0) or (u < PWM_MIN and e < 0)
                               or (i_prod >  I_TERM_LIM and e > 0)
                               or (i_prod < -I_TERM_LIM and e < 0);
                        if not stop_i then
                            iacc <= iacc_new;
                        end if;
                    end if;

                    sensor_i <= fault_l;
                    if fault_l = '0' then pv_out <= pv_l; end if;
                    state <= S_OUT;

                when S_OUT =>
                    done  <= '1';
                    state <= S_IDLE;
            end case;
        end if;
    end process;

end architecture rtl;
