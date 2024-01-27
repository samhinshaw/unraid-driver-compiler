cmd_/drivers/Linux/Module.symvers :=  sed 's/ko$$/o/'  /drivers/Linux/modules.order | scripts/mod/modpost      -o /drivers/Linux/Module.symvers -e -i Module.symvers -T - 
