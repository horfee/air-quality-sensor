#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

HOMEKIT_PATH ?= $IDF_PATH/../esp-homekit-sdk
COMMON_COMPONENT_PATH ?= $HOMEKIT_PATH/examples/common

PROJECT_NAME := air-quality-sensor
EXTRA_COMPONENT_DIRS += $(HOMEKIT_PATH)/components/
EXTRA_COMPONENT_DIRS += $(HOMEKIT_PATH)/components/homekit
EXTRA_COMPONENT_DIRS += $(COMMON_COMPONENT_PATH)
EXTRA_COMPONENT_DIRS += ./components
#EXTRA_COMPONENT_DIRS += $(HOMEKIT_PATH)/../esp-idf-lib/components

include $(IDF_PATH)/make/project.mk

