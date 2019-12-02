library verilog;
use verilog.vl_types.all;
entity hazard_detector is
    port(
        reset           : in     vl_logic;
        EX_memread      : in     vl_logic;
        EX_pcsrc        : in     vl_logic;
        EX_jump         : in     vl_logic;
        EX_jr           : in     vl_logic;
        EX_rt           : in     vl_logic_vector(4 downto 0);
        ID_rs           : in     vl_logic_vector(4 downto 0);
        ID_rt           : in     vl_logic_vector(4 downto 0);
        stall           : out    vl_logic;
        flush           : out    vl_logic
    );
end hazard_detector;
