;::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: File: gateway.s
;: Company: Sunco Systems Inc.
;: Author: Marlin Unruh
;: Date: unknown
;: Compiler: AVR-GCC 3.4.5 WinAVR 20060125
;: File Description: bridge between application calls to the functions in
;:    the bootloader area
;::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: The following memory is allocated in the .data section
;: This locates the bytes at 0x0060, so when things are
;: compiled later it will not change their location.
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.global wr_buff
.global rd_buff
.global tmp_buff
.global flag_mcp
.section .data
wr_buff:
.byte 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
.byte 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
rd_buff:
.byte 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
.byte 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
tmp_buff:
.byte 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
flag_mcp:
.byte 0
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: The scheme is to have these functions defined in the firmware
;: section and call fixed functions at the end of the bootloader
;: section. The functions at end of the bootloader section then
;: calls functions in the bootloader. This is to make the bootloader
;: code editable without trashing the address location from the
;: firmware section.
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.section .text
;::::< bootloader >::::
.global bootloader
.func bootloader
bootloader:
      call _bootloader
      ret
.endfunc
;::::< ini_ddr_port >::::
.global ini_ddr_port
.func ini_ddr_port
ini_ddr_port:
      call _ini_ddr_port
      ret
.endfunc
;::::< ini_spi >::::
.global ini_spi_port
.func ini_spi_port
ini_spi_port:
      call _ini_spi_port
      ret
.endfunc
;::::< mcp_reset >::::
.global mcp_reset
.func mcp_reset
mcp_reset:
      call _mcp_reset
      ret
.endfunc
;::::< mcp_baud >::::
.global mcp_baud
.func mcp_baud
mcp_baud:
      call _mcp_baud
      ret
.endfunc
;::::< mcp_filters >::::
.global mcp_filters
.func mcp_filters
mcp_filters:
      call _mcp_filters
      ret
.endfunc
;::::< mcp_ctrl >::::
.global mcp_ctrl
.func mcp_ctrl
mcp_ctrl:
      call _mcp_ctrl
      ret
.endfunc
;::::< mcp_config >::::
.global mcp_config
.func mcp_config
mcp_config:
      call _mcp_config
      ret
.endfunc
;::::< mcp_error_handler >::::
.global mcp_error_handler
.func mcp_error_handler
mcp_error_handler:
      call _mcp_error_handler
      ret
.endfunc
;::::< mcp_get_msg >::::
.global mcp_get_msg
.func mcp_get_msg
mcp_get_msg:
      call _mcp_get_msg
      ret
.endfunc
;::::< mcp_put_msg >::::
.global mcp_put_msg
.func mcp_put_msg
mcp_put_msg:
      call _mcp_put_msg
      ret
.endfunc
;::::< spi_transfer >::::
.global spi_transfer
.func spi_transfer
spi_transfer:
      call _spi_transfer
      ret
.endfunc
;::::< spi_mcp_bit_modify >::::
.global spi_mcp_bit_modify
.func spi_mcp_bit_modify
spi_mcp_bit_modify:
      call _spi_mcp_bit_modify
      ret
.endfunc
;::::< spi_mcp_read >::::
.global spi_mcp_read
.func spi_mcp_read
spi_mcp_read:
      call _spi_mcp_read
      ret
.endfunc
;::::< spi_mcp_write >::::
.global spi_mcp_write
.func spi_mcp_write
spi_mcp_write:
      call _spi_mcp_write
      ret
.endfunc
;::::< spi_mcp_seq_read >::::
.global spi_mcp_seq_read
.func spi_mcp_seq_read
spi_mcp_seq_read:
      call _spi_mcp_seq_read
      ret
.endfunc
;::::< spi_mcp_seq_write >::::
.global spi_mcp_seq_write
.func spi_mcp_seq_write
spi_mcp_seq_write:
      call _spi_mcp_seq_write
      ret
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: CAUTION - CAUTION - CAUTION - CAUTION - CAUTION - CAUTION - CAUTION
;:
;: The following code should not be re-arranged after the first
;: release. More function calls can be added to the end without
;: breaking the bootloader.
;:
;: CAUTION - CAUTION - CAUTION - CAUTION - CAUTION - CAUTION - CAUTION
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.section .gateway, "ax", @progbits
; this jump must stay here
      rjmp _bootloader_
;::::< _bootloader >::::
.global _bootloader
.func _bootloader
_bootloader:
      rcall _bootloader_
      ret
.endfunc
;::::< _ini_ddr_port >::::
.global _ini_ddr_port
.func _ini_ddr_port
_ini_ddr_port:
      rcall _ini_ddr_port_
      ret
.endfunc
;::::< _ini_spi >::::
.global _ini_spi_port
.func _ini_spi_port
_ini_spi_port:
      rcall _ini_spi_port_
      ret
.endfunc
;::::< _mcp_reset >::::
.global _mcp_reset
.func _mcp_reset
_mcp_reset:
      rcall _mcp_reset_
      ret
.endfunc
;::::< _mcp_baud >::::
.global _mcp_baud
.func _mcp_baud
_mcp_baud:
      rcall _mcp_baud_
      ret
.endfunc
;::::< _mcp_filters >::::
.global _mcp_filters
.func _mcp_filters
_mcp_filters:
      rcall _mcp_filters_
      ret
.endfunc
;::::< _mcp_ctrl >::::
.global _mcp_ctrl
.func _mcp_ctrl
_mcp_ctrl:
      rcall _mcp_ctrl_
      ret
.endfunc
;::::< _mcp_config >::::
.global _mcp_config
.func _mcp_config
_mcp_config:
      rcall _mcp_config_
      ret
.endfunc
;::::< _mcp_error_handler >::::
.global _mcp_error_handler
.func _mcp_error_handler
_mcp_error_handler:
      rcall _mcp_error_handler_
      ret
.endfunc
;::::< _mcp_get_msg >::::
.global _mcp_get_msg
.func _mcp_get_msg
_mcp_get_msg:
      rcall _mcp_get_msg_
      ret
.endfunc
;::::< _mcp_put_msg >::::
.global _mcp_put_msg
.func _mcp_put_msg
_mcp_put_msg:
      rcall _mcp_put_msg_
      ret
.endfunc
;::::< _spi_transfer >::::
.global _spi_transfer
.func _spi_transfer
_spi_transfer:
      rcall _spi_transfer_
      ret
.endfunc
;::::< _spi_mcp_bit_modify >::::
.global _spi_mcp_bit_modify
.func _spi_mcp_bit_modify
_spi_mcp_bit_modify:
      rcall _spi_mcp_bit_modify_
      ret
.endfunc
;::::< _spi_mcp_read >::::
.global _spi_mcp_read
.func _spi_mcp_read
_spi_mcp_read:
      rcall _spi_mcp_read_
      ret
.endfunc
;::::< _spi_mcp_write >::::
.global _spi_mcp_write
.func _spi_mcp_write
_spi_mcp_write:
      rcall _spi_mcp_write_
      ret
.endfunc
;::::< _spi_mcp_seq_read >::::
.global _spi_mcp_seq_read
.func _spi_mcp_seq_read
_spi_mcp_seq_read:
      rcall _spi_mcp_seq_read_
      ret
.endfunc
;::::< _spi_mcp_seq_write >::::
.global _spi_mcp_seq_write
.func _spi_mcp_seq_write
_spi_mcp_seq_write:
      rcall _spi_mcp_seq_write_
      ret
.endfunc

