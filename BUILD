load("@aspect_rules_js//npm:defs.bzl", "npm_link_package")
load("@aspect_rules_ts//ts:defs.bzl", "ts_config")
load("@bazel_gazelle//:def.bzl", "gazelle")
load("@npm//:defs.bzl", "npm_link_all_packages")
load("@rules_license//rules:license.bzl", "license")

# Link npm packages
npm_link_all_packages(name = "node_modules")

exports_files([
    "tsconfig.json",
    "tsconfig.node.json",
    "rollup.config.js",
    # Expose .clang-format so that the static flatbuffer codegen can format its files nicely.
    ".clang-format",
])

license(
    name = "license",
    license_kinds = ["@rules_license//licenses/spdx:Apache-2.0"],
    license_text = "LICENSE.txt",
)

# The root repo tsconfig
ts_config(
    name = "tsconfig",
    src = "tsconfig.json",
    visibility = ["//visibility:public"],
)

ts_config(
    name = "tsconfig.node",
    src = "tsconfig.node.json",
    visibility = ["//visibility:public"],
    deps = [":tsconfig"],
)

npm_link_package(
    name = "node_modules/flatbuffers",
    src = "@com_github_google_flatbuffers//ts:flatbuffers",
)

npm_link_package(
    name = "node_modules/flatbuffers_reflection",
    src = "@com_github_google_flatbuffers//reflection:flatbuffers_reflection",
)

# gazelle:prefix github.com/RealtimeRoboticsGroup/aos
# gazelle:build_file_name BUILD
# gazelle:proto disable
# gazelle:go_generate_proto false
# gazelle:exclude third_party
# gazelle:exclude external
# gazelle:resolve go github.com/google/flatbuffers/go @com_github_google_flatbuffers//go:go_default_library
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/build_tests/fbs //build_tests:test_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/error_response //scouting/webserver/requests/messages:error_response_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_notes //scouting/webserver/requests/messages:submit_notes_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_notes_response //scouting/webserver/requests/messages:submit_notes_response_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_2024_data_scouting_response //scouting/webserver/requests/messages:request_2024_data_scouting_response_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_2024_data_scouting //scouting/webserver/requests/messages:request_2024_data_scouting_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_matches_for_team_response //scouting/webserver/requests/messages:request_matches_for_team_response_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_matches_for_team //scouting/webserver/requests/messages:request_matches_for_team_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_notes_for_team_response //scouting/webserver/requests/messages:request_notes_for_team_response_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_notes_for_team //scouting/webserver/requests/messages:request_notes_for_team_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_all_matches_response //scouting/webserver/requests/messages:request_all_matches_response_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_all_matches //scouting/webserver/requests/messages:request_all_matches_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_all_notes_response //scouting/webserver/requests/messages:request_all_notes_response_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_all_notes //scouting/webserver/requests/messages:request_all_notes_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_all_driver_rankings_response //scouting/webserver/requests/messages:request_all_driver_rankings_response_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_all_driver_rankings //scouting/webserver/requests/messages:request_all_driver_rankings_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/refresh_match_list //scouting/webserver/requests/messages:refresh_match_list_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/refresh_match_list_response //scouting/webserver/requests/messages:refresh_match_list_response_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_shift_schedule //scouting/webserver/requests/messages:request_shift_schedule_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_shift_schedule_response //scouting/webserver/requests/messages:request_shift_schedule_response_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_2024_actions //scouting/webserver/requests/messages:submit_2024_actions_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_2024_actions_response //scouting/webserver/requests/messages:submit_2024_actions_response_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_shift_schedule //scouting/webserver/requests/messages:submit_shift_schedule_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_shift_schedule_response //scouting/webserver/requests/messages:submit_shift_schedule_response_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_driver_ranking //scouting/webserver/requests/messages:submit_driver_ranking_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_driver_ranking_response //scouting/webserver/requests/messages:submit_driver_ranking_response_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/delete_2024_data_scouting //scouting/webserver/requests/messages:delete_2024_data_scouting_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/delete_2024_data_scouting_response //scouting/webserver/requests/messages:delete_2024_data_scouting_response_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_pit_image //scouting/webserver/requests/messages:submit_pit_image_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_pit_image_response //scouting/webserver/requests/messages:submit_pit_image_response_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_pit_images //scouting/webserver/requests/messages:request_pit_images_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_pit_images_response //scouting/webserver/requests/messages:request_pit_images_response_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_all_pit_images //scouting/webserver/requests/messages:request_all_pit_images_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_all_pit_images_response //scouting/webserver/requests/messages:request_all_pit_images_response_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_current_scouting //scouting/webserver/requests/messages:request_current_scouting_go_fbs
# gazelle:resolve go github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_current_scouting_response //scouting/webserver/requests/messages:request_current_scouting_response_go_fbs

gazelle(
    name = "gazelle",
    visibility = ["//tools/lint:__subpackages__"],
)
