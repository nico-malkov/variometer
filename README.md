<b> Variometer v1.5 (hardware) v143 (software) </b>

The goal of this project is to create a miniature smart device for hobby paragliding: a variometer. The initial concept is as follows:

<img width="1200" height="900" alt="Image" src="https://github.com/user-attachments/assets/9f16d890-0373-4679-84ce-5fa3109c3cf5" /> <br>


<b> The current version of the project has successfully achieved its technical objectives and is being released for public access. It implements the following features: </b>

• Compact size and minimum weight of 23 grams <br>
• Power from solar cells of 170 mW at peak with consumption of 40 mW at average <br>
• IP67 rating with PTFE vented barometer membrane <br>
• Kalman filter acoustic variometer, combining data from ms5637 barometer and icm20948 IMU <br>
• Built-in capacitive touch button <br>
• nRF52840 Cortex-M4 microcontroller unit with Bluetooth capability <br>
• Secure firmware update via Bluetooth with NRF Connect android app <br>
• Integration with XCTrack android app for sourcing barometer data <br>
• Sourcing of GPS data from ORG1410 module <br>
• Creation of txt logs <br>
• Connection to PC via USB as a mass-storage device <br>

<img width="1200" height="900" alt="Image" src="https://github.com/user-attachments/assets/c519875c-205e-4be6-afe4-5303bff4de7f" />

<img width="1200" height="900" alt="Image" src="https://github.com/user-attachments/assets/608e9c01-944b-4362-971a-2a22938422ac" /> <br>

<b> The flaws of the current design include: </b>

• No IGC-format GPS tracking in software (to-do) <br>
• Large power consumption in GPS tracking mode: up to 60 mW <br>
• Custom manufacturing of solar cells is totally unpractical even though it is possible <br>

<img width="1200" height="900" alt="Image" src="https://github.com/user-attachments/assets/e6dd1ce5-97e1-4be9-a3ac-c90a13c31758" /> <br>

<b> The project folder contains 'hardware' and 'software' subfolders: </b> <br>

1) The "Hardware" folder contains a "PCB" subfolder with the complete project in Altium Designer. To open it start with the file: <br>
variometer\hardware\PCB\v1.5 Project.PrjPcb <br>

<img width="1200" height="900" alt="Image" src="https://github.com/user-attachments/assets/295052d2-3845-4659-a885-4c4dd673485b" /> <br>

2) The 'Hardware' folder contains a 'case' subfolder with the 3D model in Solid Works. To open it start with the file: <br>
variometer\hardware\case\v1.5 Case.SLDPRT <br>

<img width="1200" height="900" alt="Image" src="https://github.com/user-attachments/assets/4d6a821e-c780-4e7a-942b-93936d614f89" /> <br>

3) The 'Software' folder contains an 'nRF5_SDK_17.1.0' subfolder with the complete project in Segger Embedded Studio. To open it start with the file: <br>
variometer\software\nRF5_SDK_17.1.0\examples\peripheral\variometer_v143\vario1d5\s140\ses\vario1d5_s140.emProject <br>

<img width="1200" height="900" alt="Image" src="https://github.com/user-attachments/assets/714a341d-aff3-4789-a179-082a5da08aa0" /> <br>

Please contact me with any questions via email nicomalkov@yandex.com <br>

<b> Cheers! </b> <br>

<img width="1200" height="900" alt="Image" src="https://github.com/user-attachments/assets/ef08861e-5c89-409e-8d31-c9aa13ebdf38" />
