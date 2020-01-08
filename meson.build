project('tps2388-init', 'c')

tps2388_init_sources = ['linux_compat.c', 'linux_compat.h', 'linux_i2c.c', 'linux_i2c.h', 'main_auto.c', 'sram_code.c', 'sram_code.h', 'sys_init.h', 'tps238x.c', 'tps238x.h', 'usci_uart.h']
executable('tps2388-init', tps2388_init_sources, install : true, install_dir : get_option('sbindir'))
