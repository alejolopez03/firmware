#!/bin/bash

# Create thing, device certificate and private key
THING_NAME_DEFAULT="00000000-0000-0000-0000-000000000000"
THING_NAME=$(uuidgen | tr '[:upper:]' '[:lower:]')
THING_TYPE="wit102"

aws iot create-thing --thing-name $THING_NAME --thing-type-name $THING_TYPE

CERT_ARN=$(aws iot create-keys-and-certificate --set-as-active --certificate-pem-outfile device.pem.crt --private-key-outfile private.pem.key --query 'certificateArn' --output text)

# Attach certificate to thing and policy to certificate
aws iot attach-thing-principal --thing-name $THING_NAME --principal $CERT_ARN
aws iot attach-principal-policy --principal $THING_NAME --principal $CERT_ARN --policy-name IOT_ACCESS

# Modify nvs.csv
NVS_CONFIG="nvs.csv"

awk -v new_value="$THING_NAME" -F ',' '
BEGIN { OFS = FS }
{
  if ($1 == "dev_id") {
    $4 = new_value
  }
  print
}
' "$NVS_CONFIG" > "${NVS_CONFIG}.tmp" && mv "${NVS_CONFIG}.tmp" "$NVS_CONFIG"

# Generate the certs binary
python $IDF_PATH/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py generate "nvs.csv" build/nvs.bin 16384

# Restore nvs.csv
awk -v new_value="$THING_NAME_DEFAULT" -F ',' '
BEGIN { OFS = FS }
{
  if ($1 == "dev_id") {
    $4 = new_value
  }
  print
}
' "$NVS_CONFIG" > "${NVS_CONFIG}.tmp" && mv "${NVS_CONFIG}.tmp" "$NVS_CONFIG"