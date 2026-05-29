
# Cgroups v2 Hierarchy

In cgroups v2, for a process running as a normal user to be able to move a process from
and to a cgroup owned by that user, the common ancestor must also be owned by that user - see
https://www.kernel.org/doc/html/latest/admin-guide/cgroup-v2.html#delegation-containment .
One way to solve this problem is to introduce intermediate cgroups owned by that user.

## Example Configuration

The following AOS application configuration JSON would result in the cgroup hierarchy shown below:

```json
{
  "applications": [
    {
      "name": "process_with_extra_group",
      "user": "aos",
      "memory_limit" : 1073741824,
      "extra_cgroups": ["extra_cgroup_b"]
    },
    {
      "name": "normal_process",
      "user": "aos",
      "memory_limit" : 2073741824,
    },
    {
      "name": "some_root_process",
      "user": "root"
    }
  ]
}
```


```mermaid
graph TD
    aos_starter["/sys/fs/cgroup/system.slice/aos-starter.service/<br/>(root:root)"]

    starterd["starterd/<br/>(root:root, starterd process)"]

    user_aos["user_aos/<br/>(aos:aos)<br/>memory.max=max"]
    user_root["user_root/<br/>(root:root)<br/>memory.max=max"]

    aos_process_with_extra_group["aos_process_with_extra_group/<br/>(aos:aos)<br/>memory.max=1073741824"]
    aos_extra_cgroup_b["aos_extra_cgroup_b/<br/>(aos:aos)<br/>memory.max=max"]
    aos_normal_process["aos_normal_process/<br/>(root:root)<br/>memory.max=2073741824"]

    aos_some_root_process["aos_some_root_process/<br/>(root:root)<br/>memory.max=max"]

    brt_startup --> starterd
    brt_startup --> user_aos
    brt_startup --> user_root

    user_aos --> aos_process_with_extra_group
    user_aos --> aos_extra_cgroup_b
    user_aos --> aos_normal_process

    user_root --> aos_some_root_process
```
