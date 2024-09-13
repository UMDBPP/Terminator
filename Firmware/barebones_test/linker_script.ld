ENTRY(reset_handler)

MEMORY
{
  RAM    (xrw)    : ORIGIN = 0x20000000,   LENGTH = 40K
  SRAM2    (xrw)    : ORIGIN = 0x10000000,   LENGTH = 8K
  FLASH    (rx)    : ORIGIN = 0x8000000,   LENGTH = 128K
}