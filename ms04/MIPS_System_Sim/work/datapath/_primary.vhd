library verilog;
use verilog.vl_types.all;
entity datapath is
    port(
        clk             : in     vl_logic;
        reset           : in     vl_logic;
        signext         : in     vl_logic;
        shiftl16        : in     vl_logic;
        memtoreg        : in     vl_logic;
        pcsrc           : in     vl_logic;
        alusrc          : in     vl_logic;
        regdst          : in     vl_logic;
        regwrite        : in     vl_logic;
        jump            : in     vl_logic;
        jr              : in     vl_logic;
        alucontrol      : in     vl_logic_vector(3 downto 0);
        zero            : out    vl_logic;
        pc              : out    vl_logic_vector(31 downto 0);
        instr           : in     vl_logic_vector(31 downto 0);
        aluout          : out    vl_logic_vector(31 downto 0);
        writedata       : out    vl_logic_vector(31 downto 0);
        readdata        : in     vl_logic_vector(31 downto 0);
        ID_instr        : out    vl_logic_vector(31 downto 0);
        ID_rs           : out    vl_logic_vector(4 downto 0);
        ID_rt           : out    vl_logic_vector(4 downto 0);
        EX_rs           : out    vl_logic_vector(4 downto 0);
        EX_rt           : out    vl_logic_vector(4 downto 0);
        MEM_rt          : out    vl_logic_vector(4 downto 0);
        MEM_writereg_candidate: out    vl_logic_vector(4 downto 0);
        WB_writereg_candidate: out    vl_logic_vector(4 downto 0);
        forwarding_a    : in     vl_logic_vector(2 downto 0);
        forwarding_b    : in     vl_logic_vector(2 downto 0);
        stall           : in     vl_logic
    );
end datapath;
