# Network Setting

![Network Setting](fr3_setup.png)

The robots have static IP addresses. The IP addresses are assigned as follows:

|        Robot         | IP Address |
| -------------------- | ---------- |
| Franka Production 3  | 10.42.0.2  |
| Franka Research 3 #1 | 10.42.0.3  |
| Franka Research 3 #2 | 10.42.0.4  |

## Enable Internet Sharing

On your Ubuntu computer go to `Settings > Network` then click the gear icon on the internet interface that is connected to the LAN. Then, go to `IPv4 > IPv4 Method` and pick `Shared to other computers`, click `Apply`.

## Enable Static IP

On your Ubuntu computer go to `Settings > Network` then click the gear icon on the internet interface that is connected to the LAN. Then, go to `IPv4 > IPv4 Method` and pick `Manual`. Then, for the `Addresses` fill in

| Address | Netmask | Gateway |
| ------- | ------- | ------- |
| 10.42.0.x | 255.255.255.0 | 10.42.0.1 |

Then, click `Apply`.
