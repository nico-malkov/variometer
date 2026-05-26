<b> Variometer v1.5 (hardware) v143 (software) </b>

The goal of this project is to create a miniature intelligent device for hobby paragliding - a variometer. The initial concept lookes like this:

<img width="3508" height="2480" alt="Image" src="https://github.com/user-attachments/assets/fed3d281-5794-4462-a570-929256c53806" /> <br>


<b> The current design succeded in achieving technical goals and made open-source. It implements such features: </b>

• Compact size and minimum weight of 23 grams <br>
• Power from solar cells of 170 mW at peak with consumption of 40 mW at avarage <br>
• IP67 rating with PTFE vented barometer membrane <br>
• Kalman filter lag-free acoustic variometer, combining data from ms5637 barometer and icm20948 IMU <br>
• Bulit-in capacitive touch button <br>
• nRF52840 Cortex-M4 microcontroller unit with Bluetooth capability <br>
• Secure firmware update via Bluetooth with NRF Connect android app <br>
• Integration with XCTrack android app for sourcing barometer data <br>
• Sourcing GPS data from ORG1410 module <br>
• Creation of txt logs <br>
• Connection to PC via USB as a mass-storage device <br>

<img width="1754" height="1240" alt="Image" src="https://github.com/user-attachments/assets/42349eb3-e6e6-4404-a622-162120f83e46" />

<img width="1754" height="1240" alt="Image" src="https://github.com/user-attachments/assets/679462e2-ab79-4772-a9fa-4167f0d289d2" /> <br>

<b> The flaws of the current design include: </b>

• No IGC-format GPS tracking in software (to-do) <br>
• Large power consumption in GPS tracking mode: up to 60 mW <br>
• Сustom manufacturing of solar cells is totally unpractical even though it is possible <br>

<img width="3904" height="2441" alt="Image" src="https://github.com/user-attachments/assets/73f59914-fcfc-4cd3-9343-1eaf10ff6a32" /> <br>

<b> The project folder contains 'hardware' and 'software' subfolders: </b> <br>

1) 'Hardware' folder contains 'PCB' subfolder with complete project in Altium Designer, to open it start with the file: <br>
variometer\hardware\PCB\v1.5 Project.PrjPcb <br>

<img width="3739" height="2809" alt="Image" src="https://github.com/user-attachments/assets/9da81bb2-82d7-449f-bd39-6070436847a0" /> <br>

2) 'Hardware' folder contains 'case' subfolder with 3D model in Solid Works, to open it start with the file: <br>
variometer\hardware\case\v1.5 Case.SLDPRT <br>

<img width="3435" height="2061" alt="Image" src="https://github.com/user-attachments/assets/8481b6c9-8b5a-4438-ab42-dad2307b90e8" /> <br>

3) 'Software' folder contains 'nRF5_SDK_17.1.0' subfolder with complete project in Segger Embedded Studio, to open it start with the file: <br>
variometer\software\nRF5_SDK_17.1.0\examples\peripheral\variometer_v143\vario1d5\s140\ses\vario1d5_s140.emProject <br>

<img width="1162" height="725" alt="Image" src="https://github.com/user-attachments/assets/9da0d65d-a800-4fee-aa0b-be3a8e470ee0" /> <br>

Please contact me with any questions via email nicomalkov@yandex.com <br>

<b> Cheers! </b> <br>

<img width="1280" height="960" alt="Image" src="https://github.com/user-attachments/assets/01c76b8c-7608-46d5-b881-e75917860a43" />
