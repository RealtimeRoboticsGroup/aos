load("@aspect_rules_js//npm:defs.bzl", "npm_package")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_go_library")
load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")

# Generated with
# (cd scouting/webserver/requests/messages/; ~/aos/tools/ts/generate_flatbuffer_ts_library.py *.fbs)
FILENAMES_AND_TS_FILES = {
    "delete_2024_data_scouting": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/delete2024-data-scouting.ts",
    ],
    "delete_2024_data_scouting_response": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/delete2024-data-scouting-response.ts",
    ],
    "error_response": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/error-response.ts",
    ],
    "request_2024_data_scouting": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/request2024-data-scouting.ts",
    ],
    "request_2024_data_scouting_response": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/stats2024.ts",
        "scouting/webserver/requests/request2024-data-scouting-response.ts",
    ],
    "request_all_driver_rankings": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/request-all-driver-rankings.ts",
    ],
    "request_all_driver_rankings_response": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/ranking.ts",
        "scouting/webserver/requests/request-all-driver-rankings-response.ts",
    ],
    "request_all_matches": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/request-all-matches.ts",
    ],
    "request_all_matches_response": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/scouted-level.ts",
        "scouting/webserver/requests/match.ts",
        "scouting/webserver/requests/request-all-matches-response.ts",
    ],
    "request_all_notes": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/request-all-notes.ts",
    ],
    "request_all_notes_response": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/note.ts",
        "scouting/webserver/requests/request-all-notes-response.ts",
    ],
    "request_all_pit_images": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/request-all-pit-images.ts",
    ],
    "request_all_pit_images_response": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/pit-image.ts",
        "scouting/webserver/requests/request-all-pit-images-response.ts",
    ],
    "request_current_scouting": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/request-current-scouting.ts",
    ],
    "request_current_scouting_response": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/collected-by.ts",
        "scouting/webserver/requests/request-current-scouting-response.ts",
    ],
    "request_notes_for_team": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/request-notes-for-team.ts",
    ],
    "request_notes_for_team_response": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/note.ts",
        "scouting/webserver/requests/request-notes-for-team-response.ts",
    ],
    "request_pit_images": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/request-pit-images.ts",
    ],
    "request_pit_images_response": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/pit-image.ts",
        "scouting/webserver/requests/request-pit-images-response.ts",
    ],
    "request_shift_schedule": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/request-shift-schedule.ts",
    ],
    "request_shift_schedule_response": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/match-assignment.ts",
        "scouting/webserver/requests/request-shift-schedule-response.ts",
    ],
    "submit_2024_actions": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/start-match-action.ts",
        "scouting/webserver/requests/score-type.ts",
        "scouting/webserver/requests/no-show-action.ts",
        "scouting/webserver/requests/mobility-action.ts",
        "scouting/webserver/requests/penalty-action.ts",
        "scouting/webserver/requests/pickup-note-action.ts",
        "scouting/webserver/requests/place-note-action.ts",
        "scouting/webserver/requests/robot-death-action.ts",
        "scouting/webserver/requests/stage-type.ts",
        "scouting/webserver/requests/end-match-action.ts",
        "scouting/webserver/requests/end-auto-phase-action.ts",
        "scouting/webserver/requests/end-teleop-phase-action.ts",
        "scouting/webserver/requests/action-type.ts",
        "scouting/webserver/requests/action.ts",
        "scouting/webserver/requests/submit2024-actions.ts",
    ],
    "submit_2024_actions_response": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/submit2024-actions-response.ts",
    ],
    "submit_driver_ranking": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/submit-driver-ranking.ts",
    ],
    "submit_driver_ranking_response": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/submit-driver-ranking-response.ts",
    ],
    "submit_notes": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/submit-notes.ts",
    ],
    "submit_notes_response": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/submit-notes-response.ts",
    ],
    "submit_pit_image": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/submit-pit-image.ts",
    ],
    "submit_pit_image_response": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/submit-pit-image-response.ts",
    ],
    "submit_shift_schedule": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/match-assignment.ts",
        "scouting/webserver/requests/submit-shift-schedule.ts",
    ],
    "submit_shift_schedule_response": [
        "scouting/webserver/requests.ts",
        "scouting/webserver/requests/submit-shift-schedule-response.ts",
    ],
}

FILE_NAMES = [key for key in FILENAMES_AND_TS_FILES.keys()]

filegroup(
    name = "fbs_files",
    srcs = ["%s.fbs" % name for name in FILE_NAMES],
    visibility = ["//visibility:public"],
)

[(
    flatbuffer_go_library(
        name = name + "_go_fbs",
        srcs = [name + ".fbs"],
        importpath = "github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/" + name,
        target_compatible_with = ["@platforms//cpu:x86_64"],
        visibility = ["//visibility:public"],
    ),
    flatbuffer_ts_library(
        name = name + "_ts_fbs",
        srcs = [name + ".fbs"],
        target_compatible_with = ["@platforms//cpu:x86_64"],
        ts_files = FILENAMES_AND_TS_FILES[name],
        visibility = ["//visibility:public"],
    ),
) for name in FILE_NAMES]

npm_package(
    name = "messages",
    srcs = [
        ":package.json",
    ] + [
        ":{}_ts_fbs_ts".format(lib)
        for lib in FILE_NAMES
    ],
    visibility = ["//visibility:public"],
)
