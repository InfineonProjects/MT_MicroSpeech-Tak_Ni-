source [find interface/kitprog3.cfg]
set TARGET_AP cm4_ap
source [find target/&&CY_OPEN_OCD_FILE&&]
${TARGET}.cm4 configure -rtos auto -rtos-wipe-on-reset-halt 1
psoc6 sflash_restrictions 1
