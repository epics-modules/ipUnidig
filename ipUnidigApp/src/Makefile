TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC_vxWorks += ipUnidig

DBD += devIpUnidig.dbd

ipUnidig_SRCS += ipUnidig.cc
ipUnidig_SRCS += devIpUnidig.cc
ipUnidig_SRCS += ipUnidigServer.cc

INC += ipUnidig.h
ipUnidig_LIBS += $(EPICS_BASE_IOC_LIBS)
#=============================


#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
