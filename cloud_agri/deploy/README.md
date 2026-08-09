# deploy/

Reference configuration for running this system on more than one machine.
**Nothing here is needed for the demonstration** — `mosquitto -p 1883 -v`
on one laptop is the right amount of ceremony for showing a robot drive.
These files are what the same system needs the day the Cloud and the robot
are on a network somebody else can also reach.

| file | what it is |
|---|---|
| `mosquitto.conf` | the broker: TLS on 8883, client certificates required, plain 1883 not listened on at all |
| `acl` | who may publish what — the file that stops an authenticated stranger issuing orders |

## What the transport is for, given everything is already signed

A report is sealed to the Cloud's key and signed by the robot's; a request,
a query and a reply are all signed. So what does TLS add?

Three things the application layer cannot reach:

- **Metadata.** ECIES hides a reading; it does not hide that
  `agri/v1/report/youbot-01` published 6 kB at 14:02. Who is talking, to
  whom, how often, and from where is all in the clear, and for a
  greenhouse that is a map of the operation.
- **Reachability.** Signatures make a forged order *rejected*. They do not
  make it *impossible to send*. Without an ACL, anyone who can open a TCP
  connection can publish 48 refused orders a second and fill the log.
- **The broker itself.** Nothing in the application authenticates the
  broker to the clients. Redirect the DNS name and both sides happily talk
  to whatever answers, which sees every topic and every timing.

## Making the five files

A private CA, because a public one cannot issue a certificate for a machine
called `gh.example` on a greenhouse LAN. Ten years, because rotating this
by hand once a year is a task nobody will do; see the note at the bottom.

```bash
# 1. the CA
openssl req -x509 -newkey rsa:4096 -days 3650 -nodes \
        -keyout ca.key -out ca.crt -subj "/CN=agri-ca"

# 2. the broker's certificate. CN MUST be the name the clients connect to.
openssl req -newkey rsa:2048 -nodes -keyout broker.key -out broker.csr \
        -subj "/CN=gh.example"
openssl x509 -req -in broker.csr -CA ca.crt -CAkey ca.key -CAcreateserial \
        -days 825 -out broker.crt

# 3. one per client. The CN becomes the MQTT username, so it must match
#    the ACL: "cloud", and the robot_id for each node.
for who in cloud youbot-01; do
  openssl req -newkey rsa:2048 -nodes -keyout "$who.key" -out "$who.csr" \
          -subj "/CN=$who"
  openssl x509 -req -in "$who.csr" -CA ca.crt -CAkey ca.key -CAcreateserial \
          -days 825 -out "$who.crt"
done

chmod 600 *.key
```

Then:

```bash
sudo install -m 644 ca.crt broker.crt /etc/mosquitto/certs/
sudo install -m 600 broker.key        /etc/mosquitto/certs/
sudo install -m 644 acl               /etc/mosquitto/acl
mosquitto -c deploy/mosquitto.conf -v
```

## Running the two programs against it

```bash
agri-cloud --broker gh.example \
           --broker-ca ca.crt --broker-cert cloud.crt --broker-key cloud.key

ros2 run agri_robot robot_node --ros-args \
     -p broker:=gh.example -p broker_ca:=ca.crt \
     -p broker_cert:=youbot-01.crt -p broker_key:=youbot-01.key
```

The port follows automatically: passing `--broker-ca` turns TLS on, and TLS
moves the default from 1883 to 8883. Both programs print one line saying
exactly what they negotiated, and **neither ever falls back to plaintext**
— a system that silently downgrades is worse than one with no TLS at all,
because somebody will believe it.

If the broker is reached by bare IP and the certificate names a host, add
`--broker-insecure` (`-p broker_insecure:=true`). It skips the hostname
check and nothing else. It has to be asked for explicitly, because a
silent default there would make the certificate decorative.

## Passwords

If you use `--broker-user` instead of certificates, the password is read
from `$AGRI_BROKER_PASSWORD` and **there is no flag for it**. On Linux
`/proc/<pid>/cmdline` is world-readable, so a password on the command line
is a password `ps aux` prints to anyone with a shell, and the shell history
keeps a copy. An environment variable is not a secret either, but it is
readable by the same user and root rather than by everyone logged in.

A username with no TLS is refused outright, because MQTT sends the password
in the CONNECT packet, in the clear, before anything else happens.

## What this still does not solve

**Rotation and revocation.** These certificates last 825 days and nothing
in the system checks a CRL. A compromised robot key stays valid until
somebody notices and reissues the CA. `agri/trust.py` covers the
application keys; the transport certificates are the broker's business and
would need OCSP or a CRL served alongside `mosquitto.conf`. Stated here
rather than left to be discovered.
