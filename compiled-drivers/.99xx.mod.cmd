cmd_/drivers/Linux/99xx.mod := printf '%s\n'   99xx.o | awk '!x[$$0]++ { print("/drivers/Linux/"$$0) }' > /drivers/Linux/99xx.mod
