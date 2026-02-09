#!/usr/bin/env bash

### EDIT THIS TO WHEREVER YOU'RE STORING YOU DATA ###

LOCAL_DATA_FOLDER="/home/linh/Downloads/8_DynOSAM/dynosam_pkg/fish_data/sequences/00"
LOCAL_RESULTS_FOLDER="/home/linh/Downloads/8_DynOSAM/dynosam_pkg/results"
LOCAL_DYNO_SAM_FOLDER="/home/linh/Downloads/8_DynOSAM/dynosam_pkg/DynoSAM"
LOCAL_THIRD_PARTY_DYNO_SAM_FOLDER="/home/linh/Downloads/8_DynOSAM/dynosam_pkg/extras"

# Run the script with the NEW image tag (dynosam:blackwell)
# bash create_container_base.sh \
#     dynosam:blackwell \
#     dyno_sam_blackwell \
#     "$LOCAL_DATA_FOLDER" \
#     "$LOCAL_RESULTS_FOLDER" \
#     "$LOCAL_DYNO_SAM_FOLDER" \
#     "$LOCAL_THIRD_PARTY_DYNO_SAM_FOLDER"

bash create_container_base.sh acfr_rpg/dyno_sam_cuda dyno_sam $LOCAL_DATA_FOLDER $LOCAL_RESULTS_FOLDER $LOCAL_DYNO_SAM_FOLDER $LOCAL_THIRD_PARTY_DYNO_SAM_FOLDER
