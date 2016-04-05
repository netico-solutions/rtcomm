#!/bin/sh

echo 'obj-$(CONFIG_RTCOMM) += rtcomm/' >> ../Makefile

cat >> ../Kconfig << EOF
if STAGING

source "drivers/staging/rtcomm/Kconfig

endif #STAGING
EOF

