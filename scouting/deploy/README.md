Deploying the scouting application
================================================================================
The scouting application is deployed to `scouting.realtimeroboticsgroup.org` via `bazel`:
```console
$ bazel run //scouting/deploy
(Reading database ... 119978 files and directories currently installed.)
Preparing to unpack .../aos-scouting-server_1_amd64.deb ...
Removed /etc/systemd/system/multi-user.target.wants/scouting.service.
Unpacking aos-scouting-server (1) over (1) ...
Setting up aos-scouting-server (1) ...
Created symlink /etc/systemd/system/multi-user.target.wants/scouting.service → /etc/systemd/system/scouting.service.
Connection to scouting.realtimeroboticsgroup.org closed.
```

You will need SSH access to the scouting server. You can customize the SSH host
with the `--host` argument.

The Blue Alliance API key
--------------------------------------------------------------------------------
You need to set up an API key on the scouting server so that the scraping logic
can use it. It needs to live in `/var/aos/scouting/tba_config.json` and look
as follows:
```json
{
    "api_key": "..."
}
```

Starting and stopping the application
--------------------------------------------------------------------------------
When you SSH into the scouting server, use `systemctl` to manage
`scouting.service` like any other service.
```console
$ sudo systemctl stop scouting.service
$ sudo systemctl start scouting.service
$ sudo systemctl restart scouting.service
```

Incompatible database changes
--------------------------------------------------------------------------------
When deploying a new scouting application that has incompatible changes, you
may want to clear the existing database. This can be done by also specifying
the `--clear-db` option when deploying. This option will cause all tables to be
dropped before the scouting app is deployed.
