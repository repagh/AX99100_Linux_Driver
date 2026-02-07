#!/bin/bash

# Package information
PACKAGE_NAME="ax99100x-drv"
PACKAGE_VERSION="1.0.0"
ARCHITECTURE="all"
MAINTAINER="Tony Chung <tonychung@asix.com.tw>"

# Create temporary directory structure
TEMP_DIR="$(mktemp -d)"
mkdir -p "${TEMP_DIR}/DEBIAN"
mkdir -p "${TEMP_DIR}/usr/src/${PACKAGE_NAME}-${PACKAGE_VERSION}"
mkdir -p "${TEMP_DIR}/etc/modules-load.d"

# Create control file
cat > "${TEMP_DIR}/DEBIAN/control" << EOF
Package: ${PACKAGE_NAME}
Version: ${PACKAGE_VERSION}
Architecture: all
Maintainer: ${MAINTAINER}
Depends: build-essential
Section: kernel
Priority: optional
Description: ASIX AX99100 PCIe Bridge Driver
 This package provides the ASIX AX99100 PCIe Bridge driver.
 Includes serial port, parallel port, i2c and spi functionalities.
EOF

# Create postinst script
cat > "${TEMP_DIR}/DEBIAN/postinst" << EOF
#!/bin/bash
set -e

# Unbind any existing driver
for dev in \$(lspci -D | grep "Asix.*AX99100" | cut -d' ' -f1); do
  if [ -e "/sys/bus/pci/devices/\$dev/driver" ]; then
    echo "Unbinding \$dev from current driver..."
    echo "\$dev" > "/sys/bus/pci/devices/\$dev/driver/unbind"
  fi
done

# Install the pre-compiled driver
cd /usr/src/${PACKAGE_NAME}-${PACKAGE_VERSION}
make install

exit 0
EOF

# Create prerm script
cat > "${TEMP_DIR}/DEBIAN/prerm" << EOF
#!/bin/bash
set -e

exit 0
EOF

# Set execution permissions
chmod 755 "${TEMP_DIR}/DEBIAN/postinst"
chmod 755 "${TEMP_DIR}/DEBIAN/prerm"

# Copy source code files
cp -r ./* "${TEMP_DIR}/usr/src/${PACKAGE_NAME}-${PACKAGE_VERSION}/"


# Create module load configuration
cat > "${TEMP_DIR}/etc/modules-load.d/ax99100x.conf" << EOF
ax99100x
ax99100x_pp
parport_pc
ax99100x_i2c
EOF

# Build deb package
dpkg-deb --build "${TEMP_DIR}" "${PACKAGE_NAME}_${PACKAGE_VERSION}_${ARCHITECTURE}.deb"

# Clean temporary directory
rm -rf "${TEMP_DIR}"

echo "Deb package created: ${PACKAGE_NAME}_${PACKAGE_VERSION}_${ARCHITECTURE}.deb" 