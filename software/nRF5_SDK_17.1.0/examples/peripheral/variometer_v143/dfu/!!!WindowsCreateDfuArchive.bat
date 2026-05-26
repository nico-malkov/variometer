SET ver="1.0.0"

nrfutil pkg generate --application .\pca10056\blank\ses\Output\Release\Exe\usbd_msc_ble_uart_ota_dfu.hex --application-version-string %ver% --hw-version 52 --sd-req 0x0100 --key-file dfu_private_key.pem app_v%ver%.zip

@pause
