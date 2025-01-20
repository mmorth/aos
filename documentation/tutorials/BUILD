load("//tools/build_rules:pandoc.bzl", "pandoc_html")

articles = [
    "download-code-to-the-robot",
    "getting-started",
    "git-and-gerrit-basics",
    "network_troubleshooting",
    "rio-gdb",
    "setup-pycharm-on-build-server",
]

[pandoc_html(
    name = article_name,
    src = article_name + ".md",
    target_compatible_with = ["@platforms//os:linux"],
) for article_name in articles]
