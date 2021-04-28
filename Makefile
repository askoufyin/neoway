#--------------------------------------------------------------
#               Neoway OpenLinux
#   
#--------------------------------------------------------------
APP_NAME = NeowayTest

#CROSS_COMPILER=arm-oe-linux-gnueabi-
GCC_VERSION=4.9.2

#--------------------------------------------------------------
# Include definition
#--------------------------------------------------------------
LOCAL_INC_DIR = -I./

USR_INC_DIR = -I/home/skuf1973/NeowaySDK/include
	
#--------------------------------------------------------------
# LIB
#--------------------------------------------------------------
STD_LIB_FILES =	
USR_LIB_FILES =	-L/home/skuf1973/NeowaySDK/libs -lnwy_device -lnwy_common -lnwy_service -ldl -pthread

#--------------------------------------------------------------
# Source code files
#--------------------------------------------------------------
DIRS = . lib 
#$(shell find . -maxdepth 3 -type d)
LOCAL_SRC_FILES = $(foreach dir, $(DIRS), $(wildcard $(dir)/*.c)) 
LOCAL_OBJECTS = $(notdir $(patsubst %.c, %.o,$(LOCAL_SRC_FILES)))
		
#--------------------------------------------------------------
# Compilation options
#--------------------------------------------------------------
LOCAL_CFLAGS = -march=armv7-a -mfpu=neon -g -Xlinker -Map=output.map 


.PHONY: $(APP_NAME)

$(APP_NAME) : $(LOCAL_SRC_FILES)
	$(CC) $(LOCAL_CFLAGS) $(LOCAL_INC_DIR) $(USR_INC_DIR) $(LOCAL_SRC_FILES) $(STD_LIB_FILES) $(USR_LIB_FILES)  -o $@
	rm -rf  *.o *.a
	@echo ---- finished ----

.PHONY: clean
clean:
	rm -rf $(APP_NAME)
	rm -rf  *.o *.a


# arm-oe-linux-gnueabi-gcc  -march=armv7-a -mfloat-abi=softfp -mfpu=neon --sysroot=/home/skuf1973/NeowaySDK/tool/neoway-arm-oe-linux/sysroots/armv7a-vfp-neon-oe-linux-gnueabi -march=armv7-a -mfpu=neon -g -I./ -I/home/skuf1973/NeowaySDK/include nwy_ethernet_test.c -L/home/skuf1973/NeowaySDK/libs -lnwy_device -lnwy_common -lnwy_service

.PHONY: deploy
deploy:
	@cp -v $(APP_NAME) /xchg/
	@cp -r -v xml /xchg/
	@cp post-connect.sh /xchg/
	@cp neowayhelper.conf /xchg/
	@mkdir -p /xchg/www
	@find RelAx/Web_docs/ -type f | xargs -t -I % cp % /xchg/www
	
	
	
