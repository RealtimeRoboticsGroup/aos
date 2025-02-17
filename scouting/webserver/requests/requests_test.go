package requests

import (
	"net/http"
	"reflect"
	"testing"
	"time"

	"github.com/RealtimeRoboticsGroup/aos/scouting/db"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/debug"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/delete_2024_data_scouting"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_2024_data_scouting"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_2024_data_scouting_response"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_all_driver_rankings"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_all_driver_rankings_response"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_all_matches"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_all_matches_response"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_all_notes"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_all_notes_response"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_all_pit_images"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_all_pit_images_response"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_current_scouting"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_current_scouting_response"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_notes_for_team"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_pit_images"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_pit_images_response"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_shift_schedule"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_shift_schedule_response"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_2024_actions"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_driver_ranking"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_notes"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_pit_image"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_shift_schedule"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/server"
	flatbuffers "github.com/google/flatbuffers/go"
)

type MockClock struct {
	now time.Time
}

func (mockClock MockClock) Now() time.Time {
	return mockClock.now
}

// Validates that an unhandled address results in a 404.
func Test404(t *testing.T) {
	db := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	mockClock := MockClock{now: time.Now()}
	HandleRequests(&db, scoutingServer, &mockClock)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	resp, err := http.Get("http://localhost:8080/requests/foo")
	if err != nil {
		t.Fatalf("Failed to get data: %v", err)
	}
	if resp.StatusCode != http.StatusNotFound {
		t.Fatalf("Expected error code 404, but got %d instead", resp.Status)
	}
}

// Validates that we can request the full match list.
func TestRequestAllMatches(t *testing.T) {
	db := MockDatabase{
		matches: []db.TeamMatch{
			{
				MatchNumber: 1, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 1, TeamNumber: "5",
			},
			{
				MatchNumber: 1, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 2, TeamNumber: "42",
			},
			{
				MatchNumber: 1, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 3, TeamNumber: "600",
			},
			{
				MatchNumber: 1, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 1, TeamNumber: "971",
			},
			{
				MatchNumber: 1, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 2, TeamNumber: "400",
			},
			{
				MatchNumber: 1, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 3, TeamNumber: "200",
			},
			{
				MatchNumber: 2, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 1, TeamNumber: "6",
			},
			{
				MatchNumber: 2, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 2, TeamNumber: "43",
			},
			{
				MatchNumber: 2, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 3, TeamNumber: "601",
			},
			{
				MatchNumber: 2, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 1, TeamNumber: "972",
			},
			{
				MatchNumber: 2, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 2, TeamNumber: "401",
			},
			{
				MatchNumber: 2, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 3, TeamNumber: "201",
			},
			{
				MatchNumber: 3, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 1, TeamNumber: "7",
			},
			{
				MatchNumber: 3, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 2, TeamNumber: "44",
			},
			{
				MatchNumber: 3, SetNumber: 1, CompLevel: "qm",
				Alliance: "R", AlliancePosition: 3, TeamNumber: "602",
			},
			{
				MatchNumber: 3, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 1, TeamNumber: "973",
			},
			{
				MatchNumber: 3, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 2, TeamNumber: "402",
			},
			{
				MatchNumber: 3, SetNumber: 1, CompLevel: "qm",
				Alliance: "B", AlliancePosition: 3, TeamNumber: "202",
			},
		},
		// Pretend that we have some data scouting data.
		stats2024: []db.Stats2024{
			{
				CompType: "Regular", TeamNumber: "5",
				MatchNumber: 1, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 3,
				SpeakerAuto: 2, AmpAuto: 4, NotesDroppedAuto: 1, MobilityAuto: true,
				Speaker: 0, Amp: 1, SpeakerAmplified: 2, Shuttled: 1, OutOfField: 2,
				NotesDropped: 0, Penalties: 1, TrapNote: true, Spotlight: false, AvgCycle: 233,
				Park: false, OnStage: true, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "alex",
			},
			{
				CompType: "Regular", TeamNumber: "973",
				MatchNumber: 3, SetNumber: 1, CompLevel: "qm", StartingQuadrant: 1,
				SpeakerAuto: 0, AmpAuto: 2, NotesDroppedAuto: 0, MobilityAuto: false,
				Speaker: 0, Amp: 4, SpeakerAmplified: 3, Shuttled: 0, OutOfField: 0,
				NotesDropped: 0, Penalties: 1, TrapNote: true, Spotlight: false, AvgCycle: 120,
				Park: true, OnStage: false, Harmony: false, RobotDied: true, NoShow: false, CollectedBy: "bob",
			},
		},
	}
	scoutingServer := server.NewScoutingServer()
	mockClock := MockClock{now: time.Now()}
	HandleRequests(&db, scoutingServer, mockClock)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_all_matches.RequestAllMatchesT{}).Pack(builder))

	response, err := debug.RequestAllMatches("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request all matches: ", err)
	}

	expected := request_all_matches_response.RequestAllMatchesResponseT{
		MatchList: []*request_all_matches_response.MatchT{
			// MatchNumber, SetNumber, CompLevel
			// R1, R2, R3, B1, B2, B3
			{
				1, 1, "qm",
				"5", "42", "600", "971", "400", "200",
				&request_all_matches_response.ScoutedLevelT{
					// The R1 team has already been data
					// scouted.
					true, false, false, false, false, false,
				},
			},
			{
				2, 1, "qm",
				"6", "43", "601", "972", "401", "201",
				&request_all_matches_response.ScoutedLevelT{
					false, false, false, false, false, false,
				},
			},
			{
				3, 1, "qm",
				"7", "44", "602", "973", "402", "202",
				&request_all_matches_response.ScoutedLevelT{
					// The B1 team has already been data
					// scouted.
					false, false, false, true, false, false,
				},
			},
		},
	}
	if len(expected.MatchList) != len(response.MatchList) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}
	for i, match := range expected.MatchList {
		if !reflect.DeepEqual(*match, *response.MatchList[i]) {
			t.Fatal("Expected for match", i, ":", *match, ", but got:", *response.MatchList[i])
		}
	}

}

// Validates that we can request the 2024 stats.
func TestRequest2024DataScouting(t *testing.T) {
	db := MockDatabase{
		stats2024: []db.Stats2024{
			{
				CompType: "Regular", TeamNumber: "342",
				MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 4,
				SpeakerAuto: 1, AmpAuto: 1, NotesDroppedAuto: 0, MobilityAuto: true,
				Speaker: 4, Amp: 2, SpeakerAmplified: 1, Shuttled: 0, OutOfField: 2,
				NotesDropped: 2, Penalties: 2, TrapNote: true, Spotlight: true, AvgCycle: 0,
				Park: true, OnStage: false, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "alex",
			},
			{
				CompType: "Regular", TeamNumber: "132",
				MatchNumber: 4, SetNumber: 2, CompLevel: "quals", StartingQuadrant: 0,
				SpeakerAuto: 0, AmpAuto: 0, NotesDroppedAuto: 0, MobilityAuto: false,
				Speaker: 0, Amp: 0, SpeakerAmplified: 0, Shuttled: 0, OutOfField: 1,
				NotesDropped: 0, Penalties: 0, TrapNote: false, Spotlight: false, AvgCycle: 0,
				Park: false, OnStage: false, Harmony: false, RobotDied: false, NoShow: true, CollectedBy: "jeff",
			},
			{
				CompType: "Regular", TeamNumber: "982",
				MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 2,
				SpeakerAuto: 0, AmpAuto: 0, NotesDroppedAuto: 0, MobilityAuto: false,
				Speaker: 0, Amp: 2, SpeakerAmplified: 3, Shuttled: 1, OutOfField: 0,
				NotesDropped: 1, Penalties: 0, TrapNote: false, Spotlight: true, AvgCycle: 0,
				Park: false, OnStage: true, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "george",
			},
		},
	}
	scoutingServer := server.NewScoutingServer()
	mockClock := MockClock{now: time.Now()}
	HandleRequests(&db, scoutingServer, mockClock)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_2024_data_scouting.Request2024DataScoutingT{}).Pack(builder))

	response, err := debug.Request2024DataScouting("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request all matches: ", err)
	}

	expected := request_2024_data_scouting_response.Request2024DataScoutingResponseT{
		StatsList: []*request_2024_data_scouting_response.Stats2024T{
			{
				CompType: "Regular", TeamNumber: "342",
				MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 4,
				SpeakerAuto: 1, AmpAuto: 1, NotesDroppedAuto: 0, MobilityAuto: true,
				Speaker: 4, Amp: 2, SpeakerAmplified: 1, Shuttled: 0, OutOfField: 2,
				NotesDropped: 2, Penalties: 2, TrapNote: true, Spotlight: true, AvgCycle: 0,
				Park: true, OnStage: false, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "alex",
			},
			{
				CompType: "Regular", TeamNumber: "132",
				MatchNumber: 4, SetNumber: 2, CompLevel: "quals", StartingQuadrant: 0,
				SpeakerAuto: 0, AmpAuto: 0, NotesDroppedAuto: 0, MobilityAuto: false,
				Speaker: 0, Amp: 0, SpeakerAmplified: 0, Shuttled: 0, OutOfField: 1,
				NotesDropped: 0, Penalties: 0, TrapNote: false, Spotlight: false, AvgCycle: 0,
				Park: false, OnStage: false, Harmony: false, RobotDied: false, NoShow: true, CollectedBy: "jeff",
			},
			{
				CompType: "Regular", TeamNumber: "982",
				MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 2,
				SpeakerAuto: 0, AmpAuto: 0, NotesDroppedAuto: 0, MobilityAuto: false,
				Speaker: 0, Amp: 2, SpeakerAmplified: 3, Shuttled: 1, OutOfField: 0,
				NotesDropped: 1, Penalties: 0, TrapNote: false, Spotlight: true, AvgCycle: 0,
				Park: false, OnStage: true, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "george",
			},
		},
	}
	if len(expected.StatsList) != len(response.StatsList) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}
	for i, match := range expected.StatsList {
		if !reflect.DeepEqual(*match, *response.StatsList[i]) {
			t.Fatal("Expected for stats", i, ":", *match, ", but got:", *response.StatsList[i])
		}
	}
}

// Validates that we can request the 2024 stats.
func TestConvertActionsToStat2024(t *testing.T) {
	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&submit_2024_actions.Submit2024ActionsT{
		TeamNumber:  "4244",
		MatchNumber: 3,
		SetNumber:   1,
		CompLevel:   "quals",
		ActionsList: []*submit_2024_actions.ActionT{
			{
				ActionTaken: &submit_2024_actions.ActionTypeT{
					Type: submit_2024_actions.ActionTypeStartMatchAction,
					Value: &submit_2024_actions.StartMatchActionT{
						Position: 2,
					},
				},
				Timestamp: 0,
			},
			{
				ActionTaken: &submit_2024_actions.ActionTypeT{
					Type:  submit_2024_actions.ActionTypeNoShowAction,
					Value: &submit_2024_actions.NoShowActionT{},
				},
				Timestamp: 200,
			},
			{
				ActionTaken: &submit_2024_actions.ActionTypeT{
					Type: submit_2024_actions.ActionTypePickupNoteAction,
					Value: &submit_2024_actions.PickupNoteActionT{
						Auto: true,
					},
				},
				Timestamp: 800,
			},
			{
				ActionTaken: &submit_2024_actions.ActionTypeT{
					Type: submit_2024_actions.ActionTypePlaceNoteAction,
					Value: &submit_2024_actions.PlaceNoteActionT{
						ScoreType: submit_2024_actions.ScoreTypekAMP,
						Auto:      true,
					},
				},
				Timestamp: 2000,
			},
			{
				ActionTaken: &submit_2024_actions.ActionTypeT{
					Type: submit_2024_actions.ActionTypeMobilityAction,
					Value: &submit_2024_actions.MobilityActionT{
						Mobility: true,
					},
				},
				Timestamp: 2200,
			},
			{
				ActionTaken: &submit_2024_actions.ActionTypeT{
					Type: submit_2024_actions.ActionTypePenaltyAction,
					Value: &submit_2024_actions.PenaltyActionT{
						Penalties: 5,
					},
				},
				Timestamp: 2400,
			},
			{
				ActionTaken: &submit_2024_actions.ActionTypeT{
					Type: submit_2024_actions.ActionTypePickupNoteAction,
					Value: &submit_2024_actions.PickupNoteActionT{
						Auto: false,
					},
				},
				Timestamp: 2800,
			},
			{
				ActionTaken: &submit_2024_actions.ActionTypeT{
					Type: submit_2024_actions.ActionTypePlaceNoteAction,
					Value: &submit_2024_actions.PlaceNoteActionT{
						ScoreType: submit_2024_actions.ScoreTypekSHUTTLED,
						Auto:      false,
					},
				},
				Timestamp: 3100,
			},
			{
				ActionTaken: &submit_2024_actions.ActionTypeT{
					Type: submit_2024_actions.ActionTypePickupNoteAction,
					Value: &submit_2024_actions.PickupNoteActionT{
						Auto: false,
					},
				},
				Timestamp: 3200,
			},
			{
				ActionTaken: &submit_2024_actions.ActionTypeT{
					Type: submit_2024_actions.ActionTypePlaceNoteAction,
					Value: &submit_2024_actions.PlaceNoteActionT{
						ScoreType: submit_2024_actions.ScoreTypekDROPPED,
						Auto:      false,
					},
				},
				Timestamp: 3300,
			},
			{
				ActionTaken: &submit_2024_actions.ActionTypeT{
					Type: submit_2024_actions.ActionTypeRobotDeathAction,
					Value: &submit_2024_actions.RobotDeathActionT{
						RobotDead: true,
					},
				},
				Timestamp: 3400,
			},
			{
				ActionTaken: &submit_2024_actions.ActionTypeT{
					Type: submit_2024_actions.ActionTypeRobotDeathAction,
					Value: &submit_2024_actions.RobotDeathActionT{
						RobotDead: false,
					},
				},
				Timestamp: 3450,
			},
			{
				ActionTaken: &submit_2024_actions.ActionTypeT{
					Type: submit_2024_actions.ActionTypePickupNoteAction,
					Value: &submit_2024_actions.PickupNoteActionT{
						Auto: false,
					},
				},
				Timestamp: 3500,
			},
			{
				ActionTaken: &submit_2024_actions.ActionTypeT{
					Type: submit_2024_actions.ActionTypePlaceNoteAction,
					Value: &submit_2024_actions.PlaceNoteActionT{
						ScoreType: submit_2024_actions.ScoreTypekSPEAKER_AMPLIFIED,
						Auto:      false,
					},
				},
				Timestamp: 3900,
			},
			{
				ActionTaken: &submit_2024_actions.ActionTypeT{
					Type: submit_2024_actions.ActionTypeEndMatchAction,
					Value: &submit_2024_actions.EndMatchActionT{
						StageType: submit_2024_actions.StageTypekHARMONY,
						TrapNote:  false,
						Spotlight: false,
					},
				},
				Timestamp: 4200,
			},
		},
		CompType: "Regular",
	}).Pack(builder))

	submit2024Actions := submit_2024_actions.GetRootAsSubmit2024Actions(builder.FinishedBytes(), 0)
	response, err := ConvertActionsToStat2024(submit2024Actions)

	if err != nil {
		t.Fatal("Failed to convert actions to stats: ", err)
	}

	expected := db.Stats2024{
		CompType: "Regular", TeamNumber: "4244",
		MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 2,
		SpeakerAuto: 0, AmpAuto: 1, NotesDroppedAuto: 0, MobilityAuto: true,
		Speaker: 0, Amp: 0, SpeakerAmplified: 1, Shuttled: 1, OutOfField: 0,
		NotesDropped: 1, Penalties: 5, TrapNote: false, Spotlight: false, AvgCycle: 950,
		Park: false, OnStage: false, Harmony: true, RobotDied: true, NoShow: true, CollectedBy: "",
	}

	if expected != response {
		t.Fatal("Expected ", expected, ", but got ", response)
	}
}

func TestSubmitNotes(t *testing.T) {
	database := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	mockClock := MockClock{now: time.Now()}
	HandleRequests(&database, scoutingServer, mockClock)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&submit_notes.SubmitNotesT{
		Team:           "971",
		Notes:          "Notes",
		GoodDriving:    true,
		BadDriving:     false,
		SolidPlacing:   true,
		SketchyPlacing: false,
		GoodDefense:    true,
		BadDefense:     false,
		EasilyDefended: true,
		NoShow:         false,
		MatchNumber:    4,
		CompLevel:      "qm",
		SetNumber:      1,
	}).Pack(builder))

	_, err := debug.SubmitNotes("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to submit notes: ", err)
	}

	expected := []db.NotesData{
		{
			TeamNumber:     "971",
			Notes:          "Notes",
			GoodDriving:    true,
			BadDriving:     false,
			SolidPlacing:   true,
			SketchyPlacing: false,
			GoodDefense:    true,
			BadDefense:     false,
			EasilyDefended: true,
			NoShow:         false,
			MatchNumber:    4,
			CompLevel:      "qm",
			SetNumber:      1,
		},
	}

	if !reflect.DeepEqual(database.notes, expected) {
		t.Fatal("Submitted notes did not match", expected, database.notes)
	}
}

// Validates that we can request names of peoples who are currently scouting the same team.
func TestRequestCurrentScouting(t *testing.T) {
	database := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	mockClock := MockClock{now: time.Now()}
	HandleRequests(&database, scoutingServer, mockClock)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_current_scouting.RequestCurrentScoutingT{
		TeamNumber: "971",
	}).Pack(builder))
	response, err := debug.RequestCurrentScouting("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request current scouting: ", err)
	}

	expected := request_current_scouting_response.RequestCurrentScoutingResponseT{
		CollectedBy: []*request_current_scouting_response.CollectedByT{},
	}

	if len(expected.CollectedBy) != len(response.CollectedBy) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}
	for i, collectRecord := range expected.CollectedBy {
		if !reflect.DeepEqual(*collectRecord, *response.CollectedBy[i]) {
			t.Fatal("Expected for collected by ", i, ":", *collectRecord, ", but got:", *response.CollectedBy[i])
		}
	}

	debug.Username = "george"
	builder.Finish((&request_current_scouting.RequestCurrentScoutingT{
		TeamNumber: "971",
	}).Pack(builder))
	response, err = debug.RequestCurrentScouting("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request current scouting: ", err)
	}

	expected = request_current_scouting_response.RequestCurrentScoutingResponseT{
		CollectedBy: []*request_current_scouting_response.CollectedByT{
			{"debug_cli"},
		},
	}

	if len(expected.CollectedBy) != len(response.CollectedBy) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}

	for i, collectRecord := range expected.CollectedBy {
		if !reflect.DeepEqual(*collectRecord, *response.CollectedBy[i]) {
			t.Fatal("Expected for collected by ", i, ":", *collectRecord, ", but got:", *response.CollectedBy[i])
		}
	}

	// After skipping 10 seconds ahead, the previous request from "debug_cli" should no longer appear.
	mockClock.now = mockClock.now.Add(time.Second * 10)

	builder.Finish((&request_current_scouting.RequestCurrentScoutingT{
		TeamNumber: "971",
	}).Pack(builder))
	response, err = debug.RequestCurrentScouting("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request current scouting: ", err)
	}

	expected = request_current_scouting_response.RequestCurrentScoutingResponseT{
		CollectedBy: []*request_current_scouting_response.CollectedByT{},
	}

	// Reset username for other tests.
	debug.Username = "debug_cli"
}

func TestRequestNotes(t *testing.T) {
	database := MockDatabase{
		notes: []db.NotesData{{
			TeamNumber:     "971A",
			Notes:          "Notes",
			GoodDriving:    true,
			BadDriving:     false,
			SolidPlacing:   true,
			SketchyPlacing: false,
			GoodDefense:    true,
			BadDefense:     false,
			EasilyDefended: true,
			NoShow:         false,
			MatchNumber:    4,
			CompLevel:      "qm",
			SetNumber:      1,
		}},
	}
	scoutingServer := server.NewScoutingServer()
	mockClock := MockClock{now: time.Now()}
	HandleRequests(&database, scoutingServer, mockClock)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_notes_for_team.RequestNotesForTeamT{
		Team: "971A",
	}).Pack(builder))
	response, err := debug.RequestNotes("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to submit notes: ", err)
	}

	if response.Notes[0].Data != "Notes" {
		t.Fatal("requested notes did not match", response)
	}
}

func TestSubmitPitImage(t *testing.T) {
	database := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	mockClock := MockClock{now: time.Now()}
	HandleRequests(&database, scoutingServer, mockClock)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&submit_pit_image.SubmitPitImageT{
		TeamNumber: "483A", ImagePath: "483Arobot.jpg",
		ImageData: []byte{12, 43, 54, 34, 98},
	}).Pack(builder))

	_, err := debug.SubmitPitImage("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to submit pit image: ", err)
	}

	expected := []db.PitImage{
		{
			TeamNumber: "483A", CheckSum: "177d9dc52bc25f391232e82521259c378964c068832a9178d73448ba4ac5e0b1",
			ImagePath: "483Arobot.jpg", ImageData: []byte{12, 43, 54, 34, 98},
		},
	}

	if !reflect.DeepEqual(database.images, expected) {
		t.Fatal("Submitted image did not match", expected, database.images)
	}
}

func TestRequestPitImages(t *testing.T) {
	db := MockDatabase{
		images: []db.PitImage{
			{
				TeamNumber: "932", ImagePath: "pitimage.jpg",
				ImageData: []byte{3, 34, 44, 65}, CheckSum: "abcdf",
			},
			{
				TeamNumber: "234", ImagePath: "234robot.png",
				ImageData: []byte{64, 54, 21, 21, 76, 32}, CheckSum: "egrfd",
			},
			{
				TeamNumber: "93A", ImagePath: "abcd.jpg",
				ImageData: []byte{92, 94, 10, 30, 57, 32, 32}, CheckSum: "rgegfd",
			},
		},
	}

	scoutingServer := server.NewScoutingServer()
	mockClock := MockClock{now: time.Now()}
	HandleRequests(&db, scoutingServer, mockClock)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_pit_images.RequestPitImagesT{"932"}).Pack(builder))

	response, err := debug.RequestPitImages("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request pit images: ", err)
	}

	expected := request_pit_images_response.RequestPitImagesResponseT{
		PitImageList: []*request_pit_images_response.PitImageT{
			{
				TeamNumber: "932", ImagePath: "pitimage.jpg", CheckSum: "abcdf",
			},
		},
	}

	if len(expected.PitImageList) != len(response.PitImageList) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}

	for i, pit_image := range expected.PitImageList {
		if !reflect.DeepEqual(*pit_image, *response.PitImageList[i]) {
			t.Fatal("Expected for pit image", i, ":", *pit_image, ", but got:", *response.PitImageList[i])
		}
	}
}

func TestRequestAllPitImages(t *testing.T) {
	db := MockDatabase{
		images: []db.PitImage{
			{
				TeamNumber: "32", ImagePath: "pitimage.jpg",
				ImageData: []byte{3, 43, 44, 32}, CheckSum: "cdhrj",
			},
			{
				TeamNumber: "231", ImagePath: "232robot.png",
				ImageData: []byte{64, 54, 54, 21, 76, 32}, CheckSum: "rgre",
			},
			{
				TeamNumber: "90", ImagePath: "abcd.jpg",
				ImageData: []byte{92, 94, 10, 30, 57, 32, 32}, CheckSum: "erfer",
			},
		},
	}

	scoutingServer := server.NewScoutingServer()
	mockClock := MockClock{now: time.Now()}
	HandleRequests(&db, scoutingServer, mockClock)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_all_pit_images.RequestAllPitImagesT{}).Pack(builder))

	response, err := debug.RequestAllPitImages("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request pit images: ", err)
	}

	expected := request_all_pit_images_response.RequestAllPitImagesResponseT{
		PitImageList: []*request_all_pit_images_response.PitImageT{
			{
				TeamNumber: "32", ImagePath: "pitimage.jpg", CheckSum: "cdhrj",
			},
			{
				TeamNumber: "231", ImagePath: "232robot.png", CheckSum: "rgre",
			},
			{
				TeamNumber: "90", ImagePath: "abcd.jpg", CheckSum: "erfer",
			},
		},
	}

	if len(expected.PitImageList) != len(response.PitImageList) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}

	for i, pit_image := range expected.PitImageList {
		if !reflect.DeepEqual(*pit_image, *response.PitImageList[i]) {
			t.Fatal("Expected for pit image", i, ":", *pit_image, ", but got:", *response.PitImageList[i])
		}
	}
}

func TestRequestShiftSchedule(t *testing.T) {
	db := MockDatabase{
		shiftSchedule: []db.Shift{
			{
				MatchNumber: 1,
				R1scouter:   "Bob",
				R2scouter:   "James",
				R3scouter:   "Robert",
				B1scouter:   "Alice",
				B2scouter:   "Mary",
				B3scouter:   "Patricia",
			},
			{
				MatchNumber: 2,
				R1scouter:   "Liam",
				R2scouter:   "Noah",
				R3scouter:   "Oliver",
				B1scouter:   "Emma",
				B2scouter:   "Charlotte",
				B3scouter:   "Amelia",
			},
		},
	}
	scoutingServer := server.NewScoutingServer()
	mockClock := MockClock{now: time.Now()}
	HandleRequests(&db, scoutingServer, mockClock)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_shift_schedule.RequestShiftScheduleT{}).Pack(builder))

	response, err := debug.RequestShiftSchedule("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request shift schedule: ", err)
	}

	expected := request_shift_schedule_response.RequestShiftScheduleResponseT{
		ShiftSchedule: []*request_shift_schedule_response.MatchAssignmentT{
			{
				MatchNumber: 1,
				R1Scouter:   "Bob",
				R2Scouter:   "James",
				R3Scouter:   "Robert",
				B1Scouter:   "Alice",
				B2Scouter:   "Mary",
				B3Scouter:   "Patricia",
			},
			{
				MatchNumber: 2,
				R1Scouter:   "Liam",
				R2Scouter:   "Noah",
				R3Scouter:   "Oliver",
				B1Scouter:   "Emma",
				B2Scouter:   "Charlotte",
				B3Scouter:   "Amelia",
			},
		},
	}
	if len(expected.ShiftSchedule) != len(response.ShiftSchedule) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}
	for i, match := range expected.ShiftSchedule {
		if !reflect.DeepEqual(*match, *response.ShiftSchedule[i]) {
			t.Fatal("Expected for shift schedule", i, ":", *match, ", but got:", *response.ShiftSchedule[i])
		}
	}
}

func TestSubmitShiftSchedule(t *testing.T) {
	database := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	mockClock := MockClock{now: time.Now()}
	HandleRequests(&database, scoutingServer, mockClock)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&submit_shift_schedule.SubmitShiftScheduleT{
		ShiftSchedule: []*submit_shift_schedule.MatchAssignmentT{
			{MatchNumber: 1,
				R1Scouter: "Bob",
				R2Scouter: "James",
				R3Scouter: "Robert",
				B1Scouter: "Alice",
				B2Scouter: "Mary",
				B3Scouter: "Patricia"},
		},
	}).Pack(builder))

	_, err := debug.SubmitShiftSchedule("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to submit shift schedule: ", err)
	}

	expected := []db.Shift{
		{MatchNumber: 1,
			R1scouter: "Bob",
			R2scouter: "James",
			R3scouter: "Robert",
			B1scouter: "Alice",
			B2scouter: "Mary",
			B3scouter: "Patricia"},
	}
	if !reflect.DeepEqual(expected, database.shiftSchedule) {
		t.Fatal("Expected ", expected, ", but got:", database.shiftSchedule)
	}
}

func TestSubmitDriverRanking(t *testing.T) {
	database := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	mockClock := MockClock{now: time.Now()}
	HandleRequests(&database, scoutingServer, mockClock)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&submit_driver_ranking.SubmitDriverRankingT{
		MatchNumber: 36,
		Rank1:       "1234",
		Rank2:       "1235",
		Rank3:       "1236",
	}).Pack(builder))

	_, err := debug.SubmitDriverRanking("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to submit driver ranking: ", err)
	}

	expected := []db.DriverRankingData{
		{MatchNumber: 36, Rank1: "1234", Rank2: "1235", Rank3: "1236"},
	}

	if !reflect.DeepEqual(database.driver_ranking, expected) {
		t.Fatal("Submitted notes did not match", expected, database.notes)
	}
}

// Validates that we can request the driver rankings.
func TestRequestDriverRankings(t *testing.T) {
	db := MockDatabase{
		driver_ranking: []db.DriverRankingData{
			{
				MatchNumber: 36,
				Rank1:       "1234",
				Rank2:       "1235",
				Rank3:       "1236",
			},
			{
				MatchNumber: 36,
				Rank1:       "101",
				Rank2:       "202",
				Rank3:       "303",
			},
		},
	}
	scoutingServer := server.NewScoutingServer()
	mockClock := MockClock{now: time.Now()}
	HandleRequests(&db, scoutingServer, mockClock)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_all_driver_rankings.RequestAllDriverRankingsT{}).Pack(builder))

	response, err := debug.RequestAllDriverRankings("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request all driver rankings: ", err)
	}

	expected := request_all_driver_rankings_response.RequestAllDriverRankingsResponseT{
		DriverRankingList: []*request_all_driver_rankings_response.RankingT{
			{
				MatchNumber: 36,
				Rank1:       "1234",
				Rank2:       "1235",
				Rank3:       "1236",
			},
			{
				MatchNumber: 36,
				Rank1:       "101",
				Rank2:       "202",
				Rank3:       "303",
			},
		},
	}
	if len(expected.DriverRankingList) != len(response.DriverRankingList) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}
	for i, match := range expected.DriverRankingList {
		if !reflect.DeepEqual(*match, *response.DriverRankingList[i]) {
			t.Fatal("Expected for driver ranking", i, ":", *match, ", but got:", *response.DriverRankingList[i])
		}
	}
}

// Validates that we can request all notes.
func TestRequestAllNotes(t *testing.T) {
	db := MockDatabase{
		notes: []db.NotesData{
			{
				TeamNumber:     "971",
				Notes:          "Notes",
				GoodDriving:    true,
				BadDriving:     false,
				SolidPlacing:   true,
				SketchyPlacing: false,
				GoodDefense:    true,
				BadDefense:     false,
				EasilyDefended: false,
				NoShow:         false,
				MatchNumber:    4,
				CompLevel:      "qm",
				SetNumber:      1,
			},
			{
				TeamNumber:     "972",
				Notes:          "More Notes",
				GoodDriving:    false,
				BadDriving:     false,
				SolidPlacing:   false,
				SketchyPlacing: true,
				GoodDefense:    false,
				BadDefense:     true,
				EasilyDefended: false,
				NoShow:         false,
				MatchNumber:    1,
				CompLevel:      "qm",
				SetNumber:      2,
			},
		},
	}
	scoutingServer := server.NewScoutingServer()
	mockClock := MockClock{now: time.Now()}
	HandleRequests(&db, scoutingServer, mockClock)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&request_all_notes.RequestAllNotesT{}).Pack(builder))

	response, err := debug.RequestAllNotes("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to request all notes: ", err)
	}

	expected := request_all_notes_response.RequestAllNotesResponseT{
		NoteList: []*request_all_notes_response.NoteT{
			{
				Team:           "971",
				Notes:          "Notes",
				GoodDriving:    true,
				BadDriving:     false,
				SolidPlacing:   true,
				SketchyPlacing: false,
				GoodDefense:    true,
				BadDefense:     false,
				EasilyDefended: false,
				NoShow:         false,
				MatchNumber:    4,
				CompLevel:      "qm",
				SetNumber:      1,
			},
			{
				Team:           "972",
				Notes:          "More Notes",
				GoodDriving:    false,
				BadDriving:     false,
				SolidPlacing:   false,
				SketchyPlacing: true,
				GoodDefense:    false,
				BadDefense:     true,
				EasilyDefended: false,
				NoShow:         false,
				MatchNumber:    1,
				CompLevel:      "qm",
				SetNumber:      2,
			},
		},
	}
	if len(expected.NoteList) != len(response.NoteList) {
		t.Fatal("Expected ", expected, ", but got ", *response)
	}
	for i, note := range expected.NoteList {
		if !reflect.DeepEqual(*note, *response.NoteList[i]) {
			t.Fatal("Expected for note", i, ":", *note, ", but got:", *response.NoteList[i])
		}
	}
}

func TestAddingActions2024(t *testing.T) {
	database := MockDatabase{}
	scoutingServer := server.NewScoutingServer()
	mockClock := MockClock{now: time.Now()}
	HandleRequests(&database, scoutingServer, mockClock)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&submit_2024_actions.Submit2024ActionsT{
		TeamNumber:  "3421",
		MatchNumber: 2,
		SetNumber:   1,
		CompLevel:   "quals",
		ActionsList: []*submit_2024_actions.ActionT{
			{
				ActionTaken: &submit_2024_actions.ActionTypeT{
					Type: submit_2024_actions.ActionTypePickupNoteAction,
					Value: &submit_2024_actions.PickupNoteActionT{
						Auto: true,
					},
				},
				Timestamp: 1800,
			},
			{
				ActionTaken: &submit_2024_actions.ActionTypeT{
					Type: submit_2024_actions.ActionTypePlaceNoteAction,
					Value: &submit_2024_actions.PlaceNoteActionT{
						ScoreType: submit_2024_actions.ScoreTypekSPEAKER,
						Auto:      false,
					},
				},
				Timestamp: 2500,
			},
		},
		CompType: "Prescouting",
	}).Pack(builder))

	_, err := debug.Submit2024Actions("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to submit actions: ", err)
	}

	expectedActions := []db.Action{
		{
			CompType:        "Prescouting",
			TeamNumber:      "3421",
			MatchNumber:     2,
			SetNumber:       1,
			CompLevel:       "quals",
			CollectedBy:     "debug_cli",
			CompletedAction: []byte{},
			Timestamp:       1800,
		},
		{
			CompType:        "Prescouting",
			TeamNumber:      "3421",
			MatchNumber:     2,
			SetNumber:       1,
			CompLevel:       "quals",
			CollectedBy:     "debug_cli",
			CompletedAction: []byte{},
			Timestamp:       2500,
		},
	}

	expectedStats := []db.Stats2024{
		db.Stats2024{
			CompType: "Prescouting", TeamNumber: "3421",
			MatchNumber: 2, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 0,
			SpeakerAuto: 0, AmpAuto: 0, NotesDroppedAuto: 0, MobilityAuto: false,
			Speaker: 1, Amp: 0, SpeakerAmplified: 0, Shuttled: 0, OutOfField: 0,
			NotesDropped: 0, Penalties: 0, TrapNote: false, Spotlight: false, AvgCycle: 0,
			Park: false, OnStage: false, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "debug_cli",
		},
	}

	if !reflect.DeepEqual(expectedActions, database.actions) {
		t.Fatal("Expected ", expectedActions, ", but got:", database.actions)
	}
	if !reflect.DeepEqual(expectedStats, database.stats2024) {
		t.Fatal("Expected ", expectedStats, ", but got:", database.stats2024)
	}
}

// Validates that we can delete 2024 stats.
func TestDeleteFromStats2024(t *testing.T) {
	mockClock := MockClock{now: time.Now()}
	database := MockDatabase{
		stats2024: []db.Stats2024{
			{
				CompType: "Practice", TeamNumber: "746",
				MatchNumber: 3, SetNumber: 1, CompLevel: "quals", StartingQuadrant: 2,
				SpeakerAuto: 0, AmpAuto: 1, NotesDroppedAuto: 1, MobilityAuto: true,
				Speaker: 0, Amp: 1, SpeakerAmplified: 1, Shuttled: 0, OutOfField: 2,
				NotesDropped: 0, Penalties: 1, TrapNote: true, Spotlight: false, AvgCycle: 233,
				Park: false, OnStage: false, Harmony: true, RobotDied: false, NoShow: false, CollectedBy: "alek",
			},
			{
				CompType: "Regular", TeamNumber: "244",
				MatchNumber: 5, SetNumber: 3, CompLevel: "quals", StartingQuadrant: 1,
				SpeakerAuto: 0, AmpAuto: 0, NotesDroppedAuto: 0, MobilityAuto: false,
				Speaker: 0, Amp: 0, SpeakerAmplified: 3, Shuttled: 0, OutOfField: 0,
				NotesDropped: 0, Penalties: 1, TrapNote: false, Spotlight: false, AvgCycle: 120,
				Park: false, OnStage: true, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "kacey",
			},
		},
		actions: []db.Action{
			{
				CompType:        "Practice",
				TeamNumber:      "746",
				MatchNumber:     3,
				SetNumber:       1,
				CompLevel:       "quals",
				CollectedBy:     "debug_cli",
				CompletedAction: []byte{},
				Timestamp:       2400,
			},
			{
				CompType:        "Prescouting",
				TeamNumber:      "244",
				MatchNumber:     5,
				SetNumber:       3,
				CompLevel:       "quals",
				CollectedBy:     "debug_cli",
				CompletedAction: []byte{},
				Timestamp:       1009,
			},
		},
	}
	scoutingServer := server.NewScoutingServer()
	HandleRequests(&database, scoutingServer, mockClock)
	scoutingServer.Start(8080)
	defer scoutingServer.Stop()

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&delete_2024_data_scouting.Delete2024DataScoutingT{
		CompLevel:   "quals",
		MatchNumber: 3,
		SetNumber:   1,
		TeamNumber:  "746",
	}).Pack(builder))

	_, err := debug.Delete2024DataScouting("http://localhost:8080", builder.FinishedBytes())
	if err != nil {
		t.Fatal("Failed to delete from data scouting 2024", err)
	}

	expectedActions := []db.Action{
		{
			CompType:        "Prescouting",
			TeamNumber:      "244",
			MatchNumber:     5,
			SetNumber:       3,
			CompLevel:       "quals",
			CollectedBy:     "debug_cli",
			CompletedAction: []byte{},
			Timestamp:       1009,
		},
	}

	expectedStats := []db.Stats2024{
		{
			CompType: "Regular", TeamNumber: "244",
			MatchNumber: 5, SetNumber: 3, CompLevel: "quals", StartingQuadrant: 1,
			SpeakerAuto: 0, AmpAuto: 0, NotesDroppedAuto: 0, MobilityAuto: false,
			Speaker: 0, Amp: 0, SpeakerAmplified: 3, Shuttled: 0, OutOfField: 0,
			NotesDropped: 0, Penalties: 1, TrapNote: false, Spotlight: false, AvgCycle: 120,
			Park: false, OnStage: true, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "kacey",
		},
	}

	if !reflect.DeepEqual(expectedActions, database.actions) {
		t.Fatal("Expected ", expectedActions, ", but got:", database.actions)
	}
	if !reflect.DeepEqual(expectedStats, database.stats2024) {
		t.Fatal("Expected ", expectedStats, ", but got:", database.stats2024)
	}
}

// A mocked database we can use for testing. Add functionality to this as
// needed for your tests.

type MockDatabase struct {
	matches        []db.TeamMatch
	notes          []db.NotesData
	shiftSchedule  []db.Shift
	driver_ranking []db.DriverRankingData
	stats2024      []db.Stats2024
	actions        []db.Action
	images         []db.PitImage
}

func (database *MockDatabase) AddToMatch(match db.TeamMatch) error {
	database.matches = append(database.matches, match)
	return nil
}

func (database *MockDatabase) AddToStats2024(stats2024 db.Stats2024) error {
	database.stats2024 = append(database.stats2024, stats2024)
	return nil
}
func (database *MockDatabase) ReturnMatches() ([]db.TeamMatch, error) {
	return database.matches, nil
}

func (database *MockDatabase) ReturnStats2024() ([]db.Stats2024, error) {
	return database.stats2024, nil
}

func (database *MockDatabase) ReturnStats2024ForTeam(teamNumber string, matchNumber int32, setNumber int32, compLevel string, compType string) ([]db.Stats2024, error) {
	var results []db.Stats2024
	for _, stats := range database.stats2024 {
		if stats.TeamNumber == teamNumber && stats.MatchNumber == matchNumber && stats.SetNumber == setNumber && stats.CompLevel == compLevel && stats.CompType == compType {
			results = append(results, stats)
		}
	}
	return results, nil
}

func (database *MockDatabase) QueryNotes(requestedTeam string) ([]string, error) {
	var results []string
	for _, data := range database.notes {
		if data.TeamNumber == requestedTeam {
			results = append(results, data.Notes)
		}
	}
	return results, nil
}

func (database *MockDatabase) AddNotes(data db.NotesData) error {
	database.notes = append(database.notes, data)
	return nil
}

func (database *MockDatabase) ReturnAllNotes() ([]db.NotesData, error) {
	return database.notes, nil
}

func (database *MockDatabase) AddToShift(data db.Shift) error {
	database.shiftSchedule = append(database.shiftSchedule, data)
	return nil
}

func (database *MockDatabase) ReturnAllShifts() ([]db.Shift, error) {
	return database.shiftSchedule, nil
}

func (database *MockDatabase) QueryAllShifts(int) ([]db.Shift, error) {
	return []db.Shift{}, nil
}

func (database *MockDatabase) QueryPitImages(requestedTeam string) ([]db.RequestedPitImage, error) {
	var results []db.RequestedPitImage
	for _, data := range database.images {
		if data.TeamNumber == requestedTeam {
			results = append(results, db.RequestedPitImage{
				TeamNumber: data.TeamNumber,
				ImagePath:  data.ImagePath,
				CheckSum:   data.CheckSum,
			})
		}
	}
	return results, nil
}

func (database *MockDatabase) AddDriverRanking(data db.DriverRankingData) error {
	database.driver_ranking = append(database.driver_ranking, data)
	return nil
}

func (database *MockDatabase) ReturnAllDriverRankings() ([]db.DriverRankingData, error) {
	return database.driver_ranking, nil
}

func (database *MockDatabase) AddAction(action db.Action) error {
	database.actions = append(database.actions, action)
	return nil
}

func (database *MockDatabase) AddPitImage(pitImage db.PitImage) error {
	database.images = append(database.images, pitImage)
	return nil
}

func (database *MockDatabase) ReturnActions() ([]db.Action, error) {
	return database.actions, nil
}

func (database *MockDatabase) ReturnPitImages() ([]db.PitImage, error) {
	return database.images, nil
}

func (database *MockDatabase) DeleteFromStats2024(compLevel_ string, matchNumber_ int32, setNumber_ int32, teamNumber_ string) error {
	for i, stat := range database.stats2024 {
		if stat.CompLevel == compLevel_ &&
			stat.MatchNumber == matchNumber_ &&
			stat.SetNumber == setNumber_ &&
			stat.TeamNumber == teamNumber_ {
			// Match found, remove the element from the array.
			database.stats2024 = append(database.stats2024[:i], database.stats2024[i+1:]...)
		}
	}
	return nil
}

func (database *MockDatabase) DeleteFromActions(compLevel_ string, matchNumber_ int32, setNumber_ int32, teamNumber_ string) error {
	for i, action := range database.actions {
		if action.CompLevel == compLevel_ &&
			action.MatchNumber == matchNumber_ &&
			action.SetNumber == setNumber_ &&
			action.TeamNumber == teamNumber_ {
			// Match found, remove the element from the array.
			database.actions = append(database.actions[:i], database.actions[i+1:]...)
		}
	}
	return nil
}
