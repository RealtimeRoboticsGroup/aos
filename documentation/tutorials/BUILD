load("//tools/build_rules:pandoc.bzl", "pandoc_html")

articles = [
    "getting-started",
    "git-and-gerrit-basics",
]

[pandoc_html(
    name = article_name,
    src = article_name + ".md",
    target_compatible_with = ["@platforms//os:linux"],
) for article_name in articles]
