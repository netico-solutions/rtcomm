#!/bin/sh

echo "Generate Makefile..."
cp -v Makefile.intree Makefile

cat >> ../Makefile << EOF
#added by RTCOMM setup script
obj-\$(CONFIG_RTCOMM) += rtcomm/
EOF

echo "Modifiying ../Kconfig..."
cat >> ../Kconfig << EOF
# Added by RTCOMM setup script
if STAGING

source "drivers/staging/rtcomm/Kconfig"

endif #STAGING
EOF

echo "Don't forget to add CONFIG_RTCOMM=m|y to kernel configuration file"
