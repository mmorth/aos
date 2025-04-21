load("@bazel_skylib//rules:common_settings.bzl", "BuildSettingInfo")

def _jinja2_template_impl(ctx):
    out = ctx.outputs.out
    parameters = dict(ctx.attr.parameters)
    parameters.update(ctx.attr.list_parameters)

    if ctx.attr.flag_parameters:
        parameters.update({
            name: flag[BuildSettingInfo].value
            for flag, name in ctx.attr.flag_parameters.items()
        })

    # For now we don't really want the user to worry about which configuration
    # to pull the file from. We don't yet have a use case for pulling the same
    # file from multiple configurations. We point Jinja at all the configuration
    # roots.
    include_dirs = depset([
        file.root.path or "."
        for file in ctx.files.includes
    ]).to_list()

    args = ctx.actions.args()
    args.add(ctx.file.src)
    args.add(json.encode(parameters))
    args.add(out)
    args.add_all(include_dirs, before_each = "--include_dir")
    if ctx.file.parameters_file:
        args.add("--replacements_file", ctx.file.parameters_file)
    args.add_all(ctx.files.filter_srcs, before_each = "--filter_file")

    ctx.actions.run(
        inputs = ctx.files.src + ctx.files.includes + ctx.files.parameters_file + ctx.files.filter_srcs,
        tools = [ctx.executable._jinja2],
        progress_message = "Generating " + out.short_path,
        outputs = [out],
        executable = ctx.executable._jinja2,
        arguments = [args],
    )

    return [DefaultInfo(files = depset([out])), OutputGroupInfo(out = depset([out]))]

jinja2_template_rule = rule(
    attrs = {
        "out": attr.output(
            mandatory = True,
            doc = """The file to generate using the template. If using the jinja2_template macro below, this will automatically be populated with the contents of the `name` parameter.""",
        ),
        "src": attr.label(
            mandatory = True,
            allow_single_file = True,
            doc = """The jinja2 template file to expand.""",
        ),
        "parameters": attr.string_dict(
            mandatory = False,
            default = {},
            doc = """The string parameters to supply to Jinja2.""",
        ),
        "flag_parameters": attr.label_keyed_string_dict(
            mandatory = False,
            default = {},
            doc = """Parameters that should be sourced from string_flag() targets.""",
            providers = [BuildSettingInfo],
        ),
        "list_parameters": attr.string_list_dict(
            mandatory = False,
            default = {},
            doc = """The string list parameters to supply to Jinja2.""",
        ),
        "parameters_file": attr.label(
            allow_single_file = True,
            doc = """A JSON file whose contents are supplied as parameters to Jinja2.""",
        ),
        "includes": attr.label_list(
            allow_files = True,
            doc = """Files which are included by the template.""",
        ),
        "filter_srcs": attr.label_list(
            allow_files = [".py"],
            doc = """Files that are sourced for filters.
Needs to have a register_filters function defined.""",
        ),
        "_jinja2": attr.label(
            default = "//tools/build_rules:jinja2_generator",
            cfg = "exec",
            executable = True,
        ),
    },
    implementation = _jinja2_template_impl,
    doc = """Expands a jinja2 template given parameters.""",
)

def jinja2_template(name, src, flag_parameters = {}, **kwargs):
    # Since the `out` field will be set to `name`, and the name for the rule must
    # differ from `out`, name the rule as the `name` plus a suffix
    rule_name = name + "_rule"

    # For consistency, the usage for flag_parameters is like this:
    #
    #   flag_parameters = {
    #       "NAME1": "//path/to:flag1",
    #       "NAME2": "//path/to:flag2",
    #   },
    #
    # But the underlying bazel rule requires them like this:
    #
    #   flag_parameters = {
    #       "//path/to:flag1": "NAME1",
    #       "//path/to:flag2": "NAME2",
    #   },
    #
    # Swap the key-value pairs around here.
    swapped_flag_parameters = {value: key for key, value in (flag_parameters or {}).items()}

    jinja2_template_rule(
        name = rule_name,
        out = name,
        src = src,
        flag_parameters = swapped_flag_parameters,
        **kwargs
    )
