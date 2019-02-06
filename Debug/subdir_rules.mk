################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
can_example.obj: ../can_example.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs810/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/ti/ccs810/ccsv8/ccs_base/arm/include" --include_path="C:/ti/simplelink_msp432e4_sdk_2_30_00_14/source" --include_path="C:/ti/ccs810/ccsv8/ccs_base/arm/include/CMSIS" --include_path="C:/Files/Patrick/Not Either/git-repos/BOLT_IV_MCU" --include_path="C:/ti/ccs810/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --advice:power=all --define=ccs --define=__MSP432E401Y__ -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="can_example.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

main.obj: ../main.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs810/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/ti/ccs810/ccsv8/ccs_base/arm/include" --include_path="C:/ti/simplelink_msp432e4_sdk_2_30_00_14/source" --include_path="C:/ti/ccs810/ccsv8/ccs_base/arm/include/CMSIS" --include_path="C:/Files/Patrick/Not Either/git-repos/BOLT_IV_MCU" --include_path="C:/ti/ccs810/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --advice:power=all --define=ccs --define=__MSP432E401Y__ -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="main.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_msp432e401y_ccs.obj: ../startup_msp432e401y_ccs.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs810/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/ti/ccs810/ccsv8/ccs_base/arm/include" --include_path="C:/ti/simplelink_msp432e4_sdk_2_30_00_14/source" --include_path="C:/ti/ccs810/ccsv8/ccs_base/arm/include/CMSIS" --include_path="C:/Files/Patrick/Not Either/git-repos/BOLT_IV_MCU" --include_path="C:/ti/ccs810/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --advice:power=all --define=ccs --define=__MSP432E401Y__ -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="startup_msp432e401y_ccs.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

system_msp432e401y.obj: ../system_msp432e401y.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs810/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/ti/ccs810/ccsv8/ccs_base/arm/include" --include_path="C:/ti/simplelink_msp432e4_sdk_2_30_00_14/source" --include_path="C:/ti/ccs810/ccsv8/ccs_base/arm/include/CMSIS" --include_path="C:/Files/Patrick/Not Either/git-repos/BOLT_IV_MCU" --include_path="C:/ti/ccs810/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --advice:power=all --define=ccs --define=__MSP432E401Y__ -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="system_msp432e401y.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

uartstdio.obj: ../uartstdio.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs810/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/ti/ccs810/ccsv8/ccs_base/arm/include" --include_path="C:/ti/simplelink_msp432e4_sdk_2_30_00_14/source" --include_path="C:/ti/ccs810/ccsv8/ccs_base/arm/include/CMSIS" --include_path="C:/Files/Patrick/Not Either/git-repos/BOLT_IV_MCU" --include_path="C:/ti/ccs810/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --advice:power=all --define=ccs --define=__MSP432E401Y__ -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="uartstdio.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


