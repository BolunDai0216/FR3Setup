# Franka Research 3 Robot Configuration Guide

This guide provides step-by-step instructions for configuring a the FR3  network settings and enabling FCI (Franka Control Interface).

## Network Configuration

**1.** Connect the ethernet cable to the network port (config port) located on the robot arm. This allows a DHCP protocol (1Gb) to generate an IP address for the robot.

**2.** Configure the computer's network settings to DHCP mode:
    
    a. Navigate to the computer's settings and choose the internet option.
    
    b. Under IPv4 settings, choose `Automatic` (DHCP should be detected automatically).
    
    c. The IPv4 address is the IP address of the host (DHCP client).
    
    d. The Default Route/DNS address is the IP address of the robot (DHCP server).

**3.** Access the robot's IP address by typing it as a URL on the web browser.

**4.** Once the page loads:

    a. Uncheck the DHCP client server under the Shop Floor Plan.

    b. Enter the IP address as `10.42.0.2`.

    c. Set the Net mask as `255.255.255.0`.

    d. Click NEXT.

**5.** Click **NEXT** again and confirm the settings.

**6.** A startup page (IP_ADDRESS/desk) will open. Unlock the robot and check if FCI is available. If not, follow the steps below to **enable FCI**.

## Enabling FCI

**1.** To install FCI, provide internet access to the robot.

**2.** Ensure the robot has access to a computer with internet access. This can be the current computer where you are able to open the Franka Interface as well as a webpage. If not , you can use another computer with only internet access as well.

**3.** On your current computer with the startup page open (IP_ADDRESS/desk), go to **Settings > Network** .

**4.** Enter the **Route Gateway** as `10.42.0.1` and the name server as `8.8.8.8`. Then click **Apply**.

**5.** Connect one end of the ethernet cable (previously attached to the robot) to the robot's control box and the other end to the computer's CPU port.

**6.** Access the robot's IP address on the computer with the Internet Access.

**7.** To check if the internet access was given to the robot, it should display **Online**.

**8.** Navigate to **Settings > Frank World** and press **Download** to get the latest features. Then click **Apply > Apply and Reboot** .

**9.** Once the settings are applied, you can reconnect the ethernet cable to your own computer (if a different computer was used for internet access) and type the IP_ADDRESS as `10.42.0.2` in the URL.

**10.** A startup page (IP_ADDRESS/desk) should open, and you now have the latest updates on the robot.

## Activating FCI

#### After unlocking the brakes of the robot , enable FCI. 
#### If it doesn't work and displays an error, try the following workaround by deleting the safety parameters.

**1.** Create a safety operator account by logging in your admin account

**2.** Go to **Settings > Users > new User**.

**3.** Make a new account with the role **safety operator**.

**4.** Log out of your admin account and log in using the credentials of your **safety operator account**.

**5.** Change the robot mode to Programming mode.

**6.** Go to **Watchman > Draft > Start editing**.

**7.** You will see two rules. The easiest thing is to delete these rules. However you could also make a rule by defining safety parameters if you wish to.

**8.** Click on **Validate Rules > Confirm Whole Safety Scenario**. Now it's confirmed.

**9.** Click **Report and Commit** on the upper left corner.

**10.** Check the **I verify that I signed the report**. Click **Commit**.

**11.** Toggle to **Commited**.
