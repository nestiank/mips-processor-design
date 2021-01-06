library verilog;
use verilog.vl_types.all;
entity aludec is
    port(
        funct           : in     vl_logic_vector(5 downto 0);
        aluop           : in     vl_logic_vector(1 downto 0);
        jr              : out    vl_logic;
        alucontrol      : out    vl_logic_vector(3 downto 0)
    );
end aludec;
