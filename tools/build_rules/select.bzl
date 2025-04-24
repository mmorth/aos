load("//tools/build_rules:clean_dep.bzl", "clean_dep")

# This file contains replacements for select where the keys have more abstract
# meanings so we can map multiple conditions to the same value easily and
# quickly find issues where something new isn't handled.
# It will also make adding ORs when it makes sense easy to do nicely.

all_cpus = [
    "amd64",
    "roborio",
    "arm64",
    "cortex-m",
    "cortex-m0plus",
    "cortex-m4f-imu",
]

"""All of the CPUs we know about."""

"""A select wrapper for CPU architectures.

Args:
  values: A mapping from architecture names (as strings) to other things.
          Currently amd64, roborio, and cortex are recognized.
          'else' is also allowed as a default.
          'arm' is allowed instead of roborio, and cortex.
Returns a select which evaluates to the correct element of values.
"""

def cpu_select(values):
    if "arm" in values:
        new_values = {}
        for cpu in values:
            if cpu != "arm":
                new_values[cpu] = values[cpu]
        new_values["arm64"] = values["arm"]
        new_values["roborio"] = values["arm"]
        new_values["cortex-m"] = values["arm"]
        new_values["cortex-m0plus"] = values["arm"]
        new_values["cortex-m4f-imu"] = values["arm"]
        values = new_values
    elif "arm32" in values:
        if "arm64" not in values:
            fail("Need to handle 'arm64' CPU if handling 'arm32' CPU.")
        new_values = {}
        for cpu in values:
            if cpu != "arm32":
                new_values[cpu] = values[cpu]
        new_values["roborio"] = values["arm32"]
        new_values["cortex-m"] = values["arm32"]
        new_values["cortex-m0plus"] = values["arm32"]
        new_values["cortex-m4f-imu"] = values["arm32"]
        values = new_values
    for cpu in all_cpus:
        if cpu not in values:
            if "else" in values:
                values[cpu] = values["else"]
            else:
                fail("Need to handle %s CPUs!" % cpu, "values")
    for key in values:
        if key not in all_cpus and key != "else":
            fail("Not sure what a %s CPU is!" % key, "values")
    return select({
        clean_dep("//tools:cpu_k8"): values["amd64"],
        clean_dep("//tools:cpu_roborio"): values["roborio"],
        clean_dep("//tools:cpu_arm64"): values["arm64"],
        clean_dep("//tools:cpu_cortex_m4f"): values["cortex-m"],
        clean_dep("//tools:cpu_cortex_m0plus"): values["cortex-m0plus"],
        clean_dep("//tools:cpu_cortex-m4f-imu"): values["cortex-m4f-imu"],
        # TODO(phil): Support this properly.
        #"@//tools:cpu_cortex_m4f_k22": values["cortex-m"],
    })

"""A select wrapper for address space sizes.

Args:
  values: A mapping from address space sizes (as strings) to other things.
Returns a select which evaluates to the correct element of values.
"""

def address_size_select(values):
    if "32" not in values:
        fail("Need to handle 32 bit addresses!", "values")
    if "64" not in values:
        fail("Need to handle 64 bit addresses!", "values")
    return select({
        clean_dep("//tools:cpu_k8"): values["64"],
        clean_dep("//tools:cpu_arm64"): values["64"],
        clean_dep("//tools:cpu_roborio"): values["32"],
        clean_dep("//tools:cpu_cortex_m4f"): values["32"],
        clean_dep("//tools:cpu_cortex_m0plus"): values["32"],
        clean_dep("//tools:cpu_cortex-m4f-imu"): values["32"],
        # TODO(phil): Support this properly.
        # clean_dep("//tools:cpu_cortex_m4f_k22"): values["32"],
    })

"""A select wrapper for compilers.

Args:
  values: A mapping from compiler names (as strings) to other things.
          Currently 'gcc' and 'clang' are recognized.
Returns a select which evaluates to the correct element of values.
"""

def compiler_select(values):
    if "gcc" not in values:
        fail("Need to handle gcc!", "values")
    if "clang" not in values:
        fail("Need to handle clang!", "values")
    return select({
        clean_dep("//tools:compiler_gcc"): values["gcc"],
        clean_dep("//tools:compiler_clang"): values["clang"],
    })
