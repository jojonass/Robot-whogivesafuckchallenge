Dashboard Server and URScript: 
No specific installation is required. Simply connect your PC to the robot using a LAN cable and make sure both are on the same network.
Use the robot’s IP address (e.g., 192.168.1.xxx) and replace it with the IP used in the sample code.
You can then run the scripts directly to control or monitor the robot.

URBasic: 
First, install the dependencies listed in the requirements.txt file using(It is recommended to install the required packages in an environment such as venv.): 
pip install -r requirements.txt

After installation, you can verify that the connection is working by running:
python test_connection.py

If the connection is successful, run:
python Example_UR_URBasic.py

to move the robot and test basic motion commands.
