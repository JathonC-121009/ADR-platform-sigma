# Security policy

## Supported versions

Security fixes are applied to the latest revision of the default branch. This
project does not currently maintain separate supported release branches.

## Reporting a vulnerability

Do not open a public issue for a suspected vulnerability. Use the repository's
**Security** tab and select **Report a vulnerability** to send the maintainers
a private report. If private vulnerability reporting is not enabled, contact a
maintainer privately and ask for a secure channel without including exploit
details in the first message.

Include the affected revision, impact, reproduction steps, and any suggested
mitigation. Do not access data, networks, vehicles, or systems that you do not
own or have explicit permission to test.

## Deployment risks

- The Flask development server exposes live camera feeds without
  authentication, authorization, or TLS.
- UDP telemetry on port 5050 is unauthenticated and unencrypted.
- The navigation process can send MAVLink movement and landing commands.
- Default controller settings and camera offsets are not safe defaults for
  every airframe.

Keep these services on a trusted, isolated network; use simulation and physical
failsafes; and do not expose them directly to the public internet.
