-- =====================================================================
-- lev_pkg.vhd
-- Paquete comun del levitador: mapa de registros, tipos y CRC MODBUS
-- (version 2: se agrega el mapa MODBUS del ESP32 y sus conversiones)
--
-- CONVENCION DE MEDIDAS (sensor ultrasonico ARRIBA del tubo):
--   PV y SP son DISTANCIAS desde el sensor hasta la pelota, en mm.
--   Mas distancia = pelota mas abajo = hay que soplar mas.
-- =====================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package lev_pkg is

    -- -----------------------------------------------------------------
    -- Mapa INTERNO de registros (reg_bank, enlace con el Arduino, PID)
    -- Unidades internas: mm y ganancias x/4096.  El ESP32 NO ve este mapa:
    -- ve el mapa MODBUS (mas abajo), que mb_regmap traduce a este.
    -- -----------------------------------------------------------------
    constant NUM_REGS  : integer := 10;

    constant REG_PV     : integer := 0;  -- Distancia medida [mm]         (L)
    constant REG_SP     : integer := 1;  -- Distancia de referencia [mm]  (L/E)
    constant REG_MV     : integer := 2;  -- Senal de control PWM 0..255   (L)
    constant REG_ERROR  : integer := 3;  -- PV - SP [mm], complemento a 2 (L)
    constant REG_KP     : integer := 4;  -- Ganancia P (x/4096)           (L/E)
    constant REG_KI     : integer := 5;  -- Ganancia I (x/4096)           (L/E)
    constant REG_KD     : integer := 6;  -- Ganancia D (x/4096)           (L/E)
    constant REG_ENABLE : integer := 7;  -- 0 = lazo detenido             (L/E)
    constant REG_STATUS : integer := 8;  -- Banderas de estado            (L)
    constant REG_BIAS   : integer := 9;  -- PWM base de sustentacion      (L/E)

    -- Bits del registro STATUS
    constant ST_ENABLED  : integer := 0;  -- Lazo habilitado
    constant ST_SENSOR   : integer := 1;  -- Falla de sensor (sin eco)
    constant ST_TIMEOUT  : integer := 2;  -- Sin mediciones del Arduino (>250 ms)
    constant ST_SAT_HIGH : integer := 3;  -- MV saturada en el maximo (255)
    constant ST_SAT_LOW  : integer := 4;  -- MV en el minimo (PWM_MIN)
    constant ST_FRAME    : integer := 5;  -- Trama del Arduino con error (retenido 0.5 s)

    -- Formato de las ganancias: valor / 2^GAIN_SHIFT
    --   KP = 4096 -> 1.0 PWM por mm de error      (maximo 65535 -> 16.0)
    --   KI en PWM por (mm * muestra),  KD en PWM por (mm/muestra)
    --   Con 12 bits fraccionarios se puede representar un KI tan chico como
    --   0.00024; con x/64 el minimo era 0.0156 (demasiado grande para este lazo).
    constant GAIN_SHIFT  : integer := 12;

    -- Valores por defecto (al encender o con reset).  PUNTO DE PARTIDA CONSERVADOR:
    -- hay que sintonizarlos con la planta real (ver procedimiento de puesta en marcha).
    constant DEF_SP      : integer := 150;   -- 150 mm desde el sensor
    constant DEF_KP      : integer := 328;   -- 0.080 PWM/mm
    constant DEF_KI      : integer := 16;    -- 0.0039 PWM/(mm*muestra)
    constant DEF_KD      : integer := 2048;  -- 0.50 PWM/(mm/muestra)
    constant DEF_ENABLE  : integer := 0;     -- ARRANCA APAGADO (se activa con "on")
    constant DEF_BIAS    : integer := 100;   -- sustentacion aproximada (calibrar)
    constant DEF_PWM_MIN : integer := 30;    -- PWM minimo al que el ventilador gira

    -- Valor de PV que envia el Arduino cuando el sensor no devuelve eco
    constant PV_FAULT   : std_logic_vector(15 downto 0) := x"FFFF";

    subtype word_t is std_logic_vector(15 downto 0);
    type reg_array_t is array (0 to NUM_REGS - 1) of word_t;

    -- -----------------------------------------------------------------
    -- Mapa MODBUS (lo que ve el maestro ESP32; holding registers 0..9)
    --   Dir Nombre  Acceso  Unidades MODBUS          <-> interno
    --    0  SP      L/E     cm x100 (0.1 mm)          SP_mm  = SP/10
    --    1  KP      L/E     PWM/cm        x100        KP_int = KP * 4.096
    --    2  KI      L/E     PWM/(cm*s)    x100        KI_int = KI * 0.2048
    --    3  KD      L/E     PWM*s/cm      x100        KD_int = KD * 81.92
    --    4  EN      L/E     0 / 1                     ENABLE
    --    5  PV      L       cm x100                   PV_mm * 10
    --    6  ERR     L       SP-PV, int16, cm x100     -(PV-SP)_mm * 10
    --    7  PWM     L       0..255                    MV
    --    8  STATUS  L       bits ST_* (igual que el interno)
    --    9  BIAS    L/E     PWM 0..255 (extra; el ESP32 lee solo 0..8)
    -- SP y PV siguen siendo DISTANCIAS sensor-pelota (sensor arriba).
    -- Ganancias en unidades fisicas por cm y por segundo, con Ts = 50 ms:
    --   KP_int/4096 [PWM/mm]          = KP/100 [PWM/cm] / 10
    --   KI_int/4096 [PWM/(mm*muestra)] = KI/100 [PWM/(cm*s)] * Ts / 10
    --   KD_int/4096 [PWM/(mm/muestra)] = KD/100 [PWM*s/cm] / (10 * Ts)
    -- Ejemplo: los valores por defecto (328, 16, 2048) equivalen en MODBUS
    -- a KP = 80 (0.80), KI = 78 (0.78), KD = 25 (0.25).
    -- Si cambia Ts hay que recalcular MBK_*.
    -- -----------------------------------------------------------------
    constant MB_NUM_REGS : integer := 10;
    constant MB_SP     : integer := 0;
    constant MB_KP     : integer := 1;
    constant MB_KI     : integer := 2;
    constant MB_KD     : integer := 3;
    constant MB_EN     : integer := 4;
    constant MB_PV     : integer := 5;
    constant MB_ERR    : integer := 6;
    constant MB_PWM    : integer := 7;
    constant MB_STATUS : integer := 8;
    constant MB_BIAS   : integer := 9;

    type mb_array_t is array (0 to MB_NUM_REGS - 1) of word_t;

    -- Limites de escritura (fuera de rango -> excepcion 03)
    constant MB_SP_MAX : integer := 10000;   -- 100.00 cm
    constant MB_KP_MAX : integer := 15999;   -- 159.99 PWM/cm     (KP_int <= 65535)
    constant MB_KD_MAX : integer := 799;     --   7.99 PWM*s/cm   (KD_int <= 65535)

    -- Conversiones: y = round(x * NUM / 2^SH), saturado a 16 bits
    constant MBK_SP_NUM : natural := 104858;    constant MBK_SP_SH : natural := 20;  -- x / 10
    constant MBK_KP_NUM : natural := 4294967;   constant MBK_KP_SH : natural := 20;  -- x * 4.096
    constant MBK_KI_NUM : natural := 214748;    constant MBK_KI_SH : natural := 20;  -- x * 0.2048
    constant MBK_KD_NUM : natural := 85899346;  constant MBK_KD_SH : natural := 20;  -- x * 81.92
    -- Inversas (exactas): KP: x*125/512   KI: x*625/128   KD: x*25/2048
    constant MBR_KP_NUM : natural := 125;       constant MBR_KP_SH : natural := 9;
    constant MBR_KI_NUM : natural := 625;       constant MBR_KI_SH : natural := 7;
    constant MBR_KD_NUM : natural := 25;        constant MBR_KD_SH : natural := 11;

    function reg_writable(addr : integer) return boolean;
    function mb_writable(addr : integer) return boolean;
    function mb_value_ok(addr : integer; data : word_t) return boolean;
    function scale_u16(x : word_t; num : natural; sh : natural) return word_t;
    function x10_sat_u(x : word_t) return word_t;        -- x*10, saturado a 65535
    function neg_x10_sat_s(x : word_t) return word_t;    -- -x*10 (con signo), saturado
    function crc16_update(crc_in : std_logic_vector(15 downto 0);
                          data   : std_logic_vector(7 downto 0)) return std_logic_vector;

end package lev_pkg;

package body lev_pkg is

    function reg_writable(addr : integer) return boolean is
    begin
        return addr = REG_SP or addr = REG_KP or addr = REG_KI or
               addr = REG_KD or addr = REG_ENABLE or addr = REG_BIAS;
    end function;

    function mb_writable(addr : integer) return boolean is
    begin
        return addr = MB_SP or addr = MB_KP or addr = MB_KI or addr = MB_KD or
               addr = MB_EN or addr = MB_BIAS;
    end function;

    function mb_value_ok(addr : integer; data : word_t) return boolean is
        variable v : integer;
    begin
        v := to_integer(unsigned(data));
        case addr is
            when MB_SP   => return v <= MB_SP_MAX;
            when MB_KP   => return v <= MB_KP_MAX;
            when MB_KD   => return v <= MB_KD_MAX;
            when MB_EN   => return v <= 1;
            when MB_BIAS => return v <= 255;
            when others  => return true;
        end case;
    end function;

    -- round(x * num / 2^sh) con saturacion a 16 bits (num constante:
    -- el sintetizador lo implementa como un multiplicador por constante)
    function scale_u16(x : word_t; num : natural; sh : natural) return word_t is
        variable p : unsigned(47 downto 0);
    begin
        p := resize(unsigned(x) * to_unsigned(num, 32), 48);
        if sh > 0 then
            p := shift_right(p + shift_left(to_unsigned(1, 48), sh - 1), sh);
        end if;
        if p > 65535 then
            return x"FFFF";
        end if;
        return std_logic_vector(p(15 downto 0));
    end function;

    function x10_sat_u(x : word_t) return word_t is
    begin
        return scale_u16(x, 10, 0);
    end function;

    function neg_x10_sat_s(x : word_t) return word_t is
        variable p : signed(20 downto 0);
    begin
        p := -resize(signed(x) * to_signed(10, 5), 21);
        if p > 32767 then
            return x"7FFF";
        elsif p < -32768 then
            return x"8000";
        end if;
        return std_logic_vector(p(15 downto 0));
    end function;

    -- CRC-16 MODBUS (polinomio invertido 0xA001, valor inicial 0xFFFF)
    function crc16_update(crc_in : std_logic_vector(15 downto 0);
                          data   : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable crc : std_logic_vector(15 downto 0);
    begin
        crc := crc_in xor (x"00" & data);
        for i in 0 to 7 loop
            if crc(0) = '1' then
                crc := ('0' & crc(15 downto 1)) xor x"A001";
            else
                crc := '0' & crc(15 downto 1);
            end if;
        end loop;
        return crc;
    end function;

end package body lev_pkg;
