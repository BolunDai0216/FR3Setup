# Networking

Before any steps, we first need to ensure that we can connect to the robots from our computers. Below is the diagram depicting the network setup we have in our lab.

```{image} ../imgs/fr3_setup.png
:alt: FR3 Network Setting
:class: bg-primary mb-1
:width: 80%
:align: center
```

<div style="height: 20px;"></div>

The robots have static IP addresses. The IP addresses are assigned as follows:

|        Robot         | IP Address |
| -------------------- | ---------- |
| Franka Production 3  | 10.42.0.2  |
| Franka Research 3 #1 | 10.42.0.3  |
| Franka Research 3 #2 | 10.42.0.4  |

## Enable Internet Sharing

For the Lambda Vector Workstation that is responsible for sharing the internet connection, we need to enable internet sharing. To do this, on your Ubuntu computer go to `Settings > Network` then click the gear icon on the internet interface that is connected to the LAN. Then, go to `IPv4 > IPv4 Method` and pick `Shared to other computers`, click `Apply`.

## Enable Static IP

To bring all the other computers into the local network, we need to create static IPs for them. To do this, on your Ubuntu computer go to `Settings > Network` then click the gear icon on the internet interface that is connected to the LAN. Then, go to `IPv4 > IPv4 Method` and pick `Manual`. Then, for the `Addresses` fill in

| Address | Netmask | Gateway |
| ------- | ------- | ------- |
| 10.42.0.x | 255.255.255.0 | 10.42.0.1 |

Then, click `Apply`.
