library verilog;
use verilog.vl_types.all;
entity controller is
    port(
        op              : in     vl_logic_vector(5 downto 0);
        funct           : in     vl_logic_vector(5 downto 0);
        zero            : in     vl_logic;
        clk             : in     vl_logic;
        reset           : in     vl_logic;
        stall           : in     vl_logic;
        flush           : in     vl_logic;
        memread         : out    vl_logic;
        EX_memread      : out    vl_logic;
        MEM_regwrite    : out    vl_logic;
        WB_regwrite     : out    vl_logic;
        signext         : out    vl_logic;
        shiftl16        : out    vl_logic;
        memtoreg        : out    vl_logic;
        memwrite        : out    vl_logic;
        pcsrc           : out    vl_logic;
        alusrc          : out    vl_logic;
        regdst          : out    vl_logic;
        regwrite        : out    vl_logic;
        jump            : out    vl_logic;
        jr              : out    vl_logic;
        WB_jump         : out    vl_logic;
        WB_jr           : out    vl_logic;
        alucontrol      : out    vl_logic_vector(3 downto 0)
    );
end controller;
