library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity polyphase_fft is
    generic (
        NCH             : integer := 16;  -- Number of channels
        NTAPS_PER_BAND : integer := 12; -- Number of taps per band
        DATA_WIDTH     : integer := 16  -- Width of input data
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;
        -- Input interface
        data_in     : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        data_valid  : in  std_logic;
        -- Output interface
        data_out_re : out std_logic_vector(DATA_WIDTH-1 downto 0);
        data_out_im : out std_logic_vector(DATA_WIDTH-1 downto 0);
        out_valid   : out std_logic
    );
end polyphase_fft;

architecture rtl of polyphase_fft is
    -- Component declarations for Xilinx IP cores
    component fir_compiler_0 is
        port (
            aclk               : in  std_logic;
            s_axis_data_tvalid : in  std_logic;
            s_axis_data_tready : out std_logic;
            s_axis_data_tdata  : in  std_logic_vector(DATA_WIDTH-1 downto 0);
            m_axis_data_tvalid : out std_logic;
            m_axis_data_tdata  : out std_logic_vector(DATA_WIDTH-1 downto 0)
        );
    end component;

    component xfft_0 is
        port (
            aclk               : in  std_logic;
            s_axis_config_tvalid : in  std_logic;
            s_axis_config_tdata  : in  std_logic_vector(15 downto 0);
            s_axis_data_tvalid : in  std_logic;
            s_axis_data_tdata  : in  std_logic_vector(2*DATA_WIDTH-1 downto 0);
            m_axis_data_tvalid : out std_logic;
            m_axis_data_tdata  : out std_logic_vector(2*DATA_WIDTH-1 downto 0)
        );
    end component;

    -- Types for channel data management
    type channel_data_array is array (0 to NCH-1) of std_logic_vector(DATA_WIDTH-1 downto 0);
    
    -- Signals for channel demux
    signal channel_data    : channel_data_array;
    signal channel_counter : integer range 0 to NCH-1;
    signal data_ready     : std_logic_vector(NCH-1 downto 0);
    
    -- Signals for FFT interface
    signal fft_in_data    : std_logic_vector(2*DATA_WIDTH-1 downto 0);
    signal fft_out_data   : std_logic_vector(2*DATA_WIDTH-1 downto 0);
    signal fft_config     : std_logic_vector(15 downto 0);
    signal fft_valid      : std_logic;

begin
    -- Channel demultiplexer
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                channel_counter <= 0;
            elsif data_valid = '1' then
                channel_data(channel_counter) <= data_in;
                if channel_counter = NCH-1 then
                    channel_counter <= 0;
                else
                    channel_counter <= channel_counter + 1;
                end if;
            end if;
        end if;
    end process;

    -- Generate FIR filters for each channel
    gen_fir: for i in 0 to NCH-1 generate
        fir_inst : fir_compiler_0
            port map (
                aclk               => clk,
                s_axis_data_tvalid => data_valid and (channel_counter = i),
                s_axis_data_tdata  => channel_data(i),
                m_axis_data_tvalid => data_ready(i),
                m_axis_data_tdata  => fft_in_data((i+1)*DATA_WIDTH-1 downto i*DATA_WIDTH)
            );
    end generate;

    -- FFT configuration
    fft_config <= x"0000"; -- Forward FFT, other config bits as needed
    
    -- FFT instance
    fft_inst : xfft_0
        port map (
            aclk                  => clk,
            s_axis_config_tvalid  => '1',
            s_axis_config_tdata   => fft_config,
            s_axis_data_tvalid    => data_ready(NCH-1),
            s_axis_data_tdata     => fft_in_data,
            m_axis_data_tvalid    => fft_valid,
            m_axis_data_tdata     => fft_out_data
        );

    -- Output assignment
    data_out_re <= fft_out_data(DATA_WIDTH-1 downto 0);
    data_out_im <= fft_out_data(2*DATA_WIDTH-1 downto DATA_WIDTH);
    out_valid   <= fft_valid;

end rtl;