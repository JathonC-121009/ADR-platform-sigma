# Security policy

## Supported versions

Security fixes are applied to the latest revision of the default branch. This
project does not currently maintain separate supported release branches.

## Reporting a vulnerability

Please do not open a public issue for a suspected vulnerability. Use the
repository's **Security** tab and select **Report a vulnerability** to send the
maintainers a private report. If private vulnerability reporting is not
enabled, contact a maintainer privately and ask for a secure reporting channel
without including exploit details in the first message.

Include the affected revision, impact, reproduction steps, and any suggested
mitigation. Do not access data or systems that you do not own or have explicit
permission to test.

## Deployment warning

The Flask development server exposes live camera feeds without authentication,
authorization, or TLS. Do not expose it directly to the public internet. Use it
on a trusted network or behind a properly configured authenticated reverse
proxy.
