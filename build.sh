#
# Section "1. General Preparation"
# 
binary_output_folder="bin"
var_source_dump="."
var_dir_pwd=$(pwd)
export ARCH=arm
export CROSS_COMPILE=aarch64-linux-gnu-


if [ -z "$LINUX_S32V234_DIR" ];
  then 
    echo "Please define LINUX_S32V234_DIR pointing to kernel directory ";
    exit 0
fi

rm -rf "${binary_output_folder}"
mkdir "${binary_output_folder}"

#
# 2. Building the Kernel Modules to test for missing pieces
#
var_dir_lkm=()
var_dir_lkm+=("libs/isp/cam_generic/kernel/build-v234ce-gnu-linux-d") # 0
var_dir_lkm+=("libs/isp/csi/kernel/build-v234ce-gnu-linux-d")         # 1
var_dir_lkm+=("libs/isp/fdma/kernel/build-v234ce-gnu-linux-d")        # 2
var_dir_lkm+=("libs/isp/h264enc/kernel/build-v234ce-gnu-linux-d")    # 3
var_dir_lkm+=("libs/isp/h264dec/kernel/build-v234ce-gnu-linux-d")     # 4
var_dir_lkm+=("libs/isp/jpegdec/kernel/build-v234ce-gnu-linux-d")     # 5
var_dir_lkm+=("libs/isp/sequencer/kernel/build-v234ce-gnu-linux-d")   # 6
var_dir_lkm+=("libs/isp/viu/kernel/build-v234ce-gnu-linux-d")         # 7
var_dir_lkm+=("libs/utils/oal/kernel/build-v234ce-gnu-linux-d")       # 8
var_dir_lkm+=("libs/apex/drivers/kernel/build-v234ce-gnu-linux-d")    # 9


set -e
# Any subsequent(*) commands which fail will cause the shell script to exit immediately


for var_index in "${!var_dir_lkm[@]}"; do
   var_dir="${var_source_dump}"/"${var_dir_lkm[${var_index}]}"

   cd "${var_dir}"
   make
   cp *.ko "${var_dir_pwd}"/"${binary_output_folder}"
   cd "${var_dir_pwd}"
done
