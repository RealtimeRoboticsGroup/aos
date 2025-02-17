package requests

import (
	"encoding/base64"
	"errors"
	"fmt"
	"io"
	"log"
	"net/http"
	"sort"
	"strconv"
	"strings"
	"time"

	"github.com/RealtimeRoboticsGroup/aos/scouting/db"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/delete_2024_data_scouting"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/delete_2024_data_scouting_response"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/error_response"
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
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_notes_for_team_response"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_pit_images"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_pit_images_response"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_shift_schedule"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/request_shift_schedule_response"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_2024_actions"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_2024_actions_response"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_driver_ranking"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_driver_ranking_response"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_notes"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_notes_response"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_pit_image"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_pit_image_response"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_shift_schedule"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/messages/submit_shift_schedule_response"
	"github.com/RealtimeRoboticsGroup/aos/scouting/webserver/server"
	flatbuffers "github.com/google/flatbuffers/go"
)

type RequestAllMatches = request_all_matches.RequestAllMatches
type RequestAllMatchesResponseT = request_all_matches_response.RequestAllMatchesResponseT
type RequestAllDriverRankings = request_all_driver_rankings.RequestAllDriverRankings
type RequestAllDriverRankingsResponseT = request_all_driver_rankings_response.RequestAllDriverRankingsResponseT
type RequestAllNotes = request_all_notes.RequestAllNotes
type RequestAllNotesResponseT = request_all_notes_response.RequestAllNotesResponseT
type Request2024DataScouting = request_2024_data_scouting.Request2024DataScouting
type Request2024DataScoutingResponseT = request_2024_data_scouting_response.Request2024DataScoutingResponseT
type SubmitNotes = submit_notes.SubmitNotes
type SubmitNotesResponseT = submit_notes_response.SubmitNotesResponseT
type SubmitPitImage = submit_pit_image.SubmitPitImage
type SubmitPitImageResponseT = submit_pit_image_response.SubmitPitImageResponseT
type RequestPitImages = request_pit_images.RequestPitImages
type RequestPitImagesResponseT = request_pit_images_response.RequestPitImagesResponseT
type RequestAllPitImages = request_all_pit_images.RequestAllPitImages
type RequestAllPitImagesResponseT = request_all_pit_images_response.RequestAllPitImagesResponseT
type RequestCurrentScouting = request_current_scouting.RequestCurrentScouting
type RequestCurrentScoutingResponseT = request_current_scouting_response.RequestCurrentScoutingResponseT
type RequestNotesForTeam = request_notes_for_team.RequestNotesForTeam
type RequestNotesForTeamResponseT = request_notes_for_team_response.RequestNotesForTeamResponseT
type RequestShiftSchedule = request_shift_schedule.RequestShiftSchedule
type RequestShiftScheduleResponseT = request_shift_schedule_response.RequestShiftScheduleResponseT
type SubmitShiftSchedule = submit_shift_schedule.SubmitShiftSchedule
type SubmitShiftScheduleResponseT = submit_shift_schedule_response.SubmitShiftScheduleResponseT
type SubmitDriverRanking = submit_driver_ranking.SubmitDriverRanking
type SubmitDriverRankingResponseT = submit_driver_ranking_response.SubmitDriverRankingResponseT
type Action2024 = submit_2024_actions.Action
type Submit2024Actions = submit_2024_actions.Submit2024Actions
type Submit2024ActionsResponseT = submit_2024_actions_response.Submit2024ActionsResponseT
type Delete2024DataScouting = delete_2024_data_scouting.Delete2024DataScouting
type Delete2024DataScoutingResponseT = delete_2024_data_scouting_response.Delete2024DataScoutingResponseT

// The interface we expect the database abstraction to conform to.
// We use an interface here because it makes unit testing easier.
type Database interface {
	AddToMatch(db.TeamMatch) error
	AddToShift(db.Shift) error
	AddToStats2024(db.Stats2024) error
	ReturnMatches() ([]db.TeamMatch, error)
	ReturnAllNotes() ([]db.NotesData, error)
	ReturnAllDriverRankings() ([]db.DriverRankingData, error)
	ReturnAllShifts() ([]db.Shift, error)
	ReturnStats2024() ([]db.Stats2024, error)
	ReturnStats2024ForTeam(teamNumber string, matchNumber int32, setNumber int32, compLevel string, compType string) ([]db.Stats2024, error)
	QueryAllShifts(int) ([]db.Shift, error)
	QueryNotes(string) ([]string, error)
	QueryPitImages(string) ([]db.RequestedPitImage, error)
	ReturnPitImages() ([]db.PitImage, error)
	AddNotes(db.NotesData) error
	AddPitImage(db.PitImage) error
	AddDriverRanking(db.DriverRankingData) error
	AddAction(db.Action) error
	DeleteFromStats2024(string, int32, int32, string) error
	DeleteFromActions(string, int32, int32, string) error
}

type Clock interface {
	Now() time.Time
}

type RealClock struct{}

func (RealClock) Now() time.Time {
	return time.Now()
}

// Handles unknown requests. Just returns a 404.
func unknown(w http.ResponseWriter, req *http.Request) {
	w.WriteHeader(http.StatusNotFound)
}

func respondWithError(w http.ResponseWriter, statusCode int, errorMessage string) {
	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&error_response.ErrorResponseT{
		ErrorMessage: errorMessage,
	}).Pack(builder))
	w.WriteHeader(statusCode)
	w.Write(builder.FinishedBytes())
}

func respondNotImplemented(w http.ResponseWriter) {
	respondWithError(w, http.StatusNotImplemented, "")
}

func parseRequest[T interface{}](w http.ResponseWriter, buf []byte, requestName string, parser func([]byte, flatbuffers.UOffsetT) *T) (*T, bool) {
	success := true
	defer func() {
		if r := recover(); r != nil {
			respondWithError(w, http.StatusBadRequest, fmt.Sprintf("Failed to parse %s: %v", requestName, r))
			success = false
		}
	}()
	result := parser(buf, 0)
	return result, success
}

// Parses the authorization information that the browser inserts into the
// headers.  The authorization follows this format:
//
//	req.Headers["Authorization"] = []string{"Basic <base64 encoded username:password>"}
func parseUsername(req *http.Request) string {
	auth, ok := req.Header["Authorization"]
	if !ok {
		return "unknown"
	}

	parts := strings.Split(auth[0], " ")
	if !(len(parts) == 2 && parts[0] == "Basic") {
		return "unknown"
	}

	info, err := base64.StdEncoding.DecodeString(parts[1])
	if err != nil {
		log.Println("ERROR: Failed to parse Basic authentication.")
		return "unknown"
	}

	loginParts := strings.Split(string(info), ":")
	if len(loginParts) != 2 {
		return "unknown"
	}
	return loginParts[0]
}

// Handles a RequestAllMaches request.
type requestAllMatchesHandler struct {
	db Database
}

// Change structure of match objects in the database(1 per team) to
// the old match structure(1 per match) that the webserver uses.
// We use the information in this struct to identify which match object
// corresponds to which old match structure object.
type MatchAssemblyKey struct {
	MatchNumber int32
	SetNumber   int32
	CompLevel   string
}

func findIndexInList(list []string, comp_level string) (int, error) {
	for index, value := range list {
		if value == comp_level {
			return index, nil
		}
	}
	return -1, errors.New(fmt.Sprint("Failed to find comp level ", comp_level, " in list ", list))
}

func (handler requestAllMatchesHandler) teamHasBeenDataScouted(key MatchAssemblyKey, teamNumber string) (bool, error) {
	stats, err := handler.db.ReturnStats2024ForTeam(
		teamNumber, key.MatchNumber, key.SetNumber, key.CompLevel, "Regular")
	if err != nil {
		return false, err
	}
	return (len(stats) > 0), nil
}

func (handler requestAllMatchesHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	_, success := parseRequest(w, requestBytes, "RequestAllMatches", request_all_matches.GetRootAsRequestAllMatches)
	if !success {
		return
	}

	matches, err := handler.db.ReturnMatches()
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to query database: ", err))
		return
	}

	assembledMatches := map[MatchAssemblyKey]request_all_matches_response.MatchT{}

	for _, match := range matches {
		key := MatchAssemblyKey{match.MatchNumber, match.SetNumber, match.CompLevel}

		// Retrieve the converted match structure we have assembled so
		// far. If we haven't started assembling one yet, then start a
		// new one.
		entry, ok := assembledMatches[key]
		if !ok {
			entry = request_all_matches_response.MatchT{
				MatchNumber: match.MatchNumber,
				SetNumber:   match.SetNumber,
				CompLevel:   match.CompLevel,
				DataScouted: &request_all_matches_response.ScoutedLevelT{},
			}
		}

		var team *string
		var dataScoutedTeam *bool

		// Fill in the field for the match that we have in in the
		// database. In the database, each match row only has 1 team
		// number.
		switch match.Alliance {
		case "R":
			switch match.AlliancePosition {
			case 1:
				team = &entry.R1
				dataScoutedTeam = &entry.DataScouted.R1
			case 2:
				team = &entry.R2
				dataScoutedTeam = &entry.DataScouted.R2
			case 3:
				team = &entry.R3
				dataScoutedTeam = &entry.DataScouted.R3
			default:
				respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Unknown red position ", strconv.Itoa(int(match.AlliancePosition)), " in match ", strconv.Itoa(int(match.MatchNumber))))
				return
			}
		case "B":
			switch match.AlliancePosition {
			case 1:
				team = &entry.B1
				dataScoutedTeam = &entry.DataScouted.B1
			case 2:
				team = &entry.B2
				dataScoutedTeam = &entry.DataScouted.B2
			case 3:
				team = &entry.B3
				dataScoutedTeam = &entry.DataScouted.B3
			default:
				respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Unknown blue position ", strconv.Itoa(int(match.AlliancePosition)), " in match ", strconv.Itoa(int(match.MatchNumber))))
				return
			}
		default:
			respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Unknown alliance ", match.Alliance, " in match ", strconv.Itoa(int(match.AlliancePosition))))
			return
		}

		*team = match.TeamNumber

		// Figure out if this team has been data scouted already.
		*dataScoutedTeam, err = handler.teamHasBeenDataScouted(key, match.TeamNumber)
		if err != nil {
			respondWithError(w, http.StatusInternalServerError, fmt.Sprint(
				"Failed to determine data scouting status for team ",
				strconv.Itoa(int(match.AlliancePosition)),
				" in match ",
				strconv.Itoa(int(match.MatchNumber)),
				err))
			return
		}

		assembledMatches[key] = entry
	}

	var response RequestAllMatchesResponseT
	for _, match := range assembledMatches {
		copied_match := match
		response.MatchList = append(response.MatchList, &copied_match)
	}

	var MATCH_TYPE_ORDERING = []string{"qm", "ef", "qf", "sf", "f"}

	err = nil
	sort.Slice(response.MatchList, func(i, j int) bool {
		if err != nil {
			return false
		}
		a := response.MatchList[i]
		b := response.MatchList[j]

		aMatchTypeIndex, err2 := findIndexInList(MATCH_TYPE_ORDERING, a.CompLevel)
		if err2 != nil {
			err = errors.New(fmt.Sprint("Comp level ", a.CompLevel, " not found in sorting list ", MATCH_TYPE_ORDERING, " : ", err2))
			return false
		}
		bMatchTypeIndex, err2 := findIndexInList(MATCH_TYPE_ORDERING, b.CompLevel)
		if err2 != nil {
			err = errors.New(fmt.Sprint("Comp level ", b.CompLevel, " not found in sorting list ", MATCH_TYPE_ORDERING, " : ", err2))
			return false
		}

		if aMatchTypeIndex < bMatchTypeIndex {
			return true
		}
		if aMatchTypeIndex > bMatchTypeIndex {
			return false
		}

		// Then sort by match number. E.g. in semi finals, all match 1 rounds
		// are done first. Then come match 2 rounds. And then, if necessary,
		// the match 3 rounds.
		aMatchNumber := a.MatchNumber
		bMatchNumber := b.MatchNumber
		if aMatchNumber < bMatchNumber {
			return true
		}
		if aMatchNumber > bMatchNumber {
			return false
		}
		// Lastly, sort by set number. I.e. Semi Final 1 Match 1 happens first.
		// Then comes Semi Final 2 Match 1. Then comes Semi Final 1 Match 2. Then
		// Semi Final 2 Match 2.
		aSetNumber := a.SetNumber
		bSetNumber := b.SetNumber
		if aSetNumber < bSetNumber {
			return true
		}
		if aSetNumber > bSetNumber {
			return false
		}
		return true
	})

	if err != nil {
		// check if error happened during sorting and notify webpage if that
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint(err))
		return
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type requestCurrentScoutingHandler struct {
	// Map that has a key of team number with a value is a map of names to timestamps
	// so there aren't duplicate timestamps for one person.
	scoutingMap map[string]map[string]time.Time
	db          Database
	clock       Clock
}

func (handler requestCurrentScoutingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "RequestCurrentScouting", request_current_scouting.GetRootAsRequestCurrentScouting)
	if !success {
		return
	}
	currentTime := handler.clock.Now()
	teamNumber := string(request.TeamNumber())
	collectedBy := parseUsername(req)

	if handler.scoutingMap[teamNumber] == nil {
		handler.scoutingMap[teamNumber] = map[string]time.Time{}
	}
	handler.scoutingMap[teamNumber][collectedBy] = currentTime
	// Delete any scout information from 10+ seconds ago.
	for team, teamMap := range handler.scoutingMap {
		for name, timeStamp := range teamMap {
			if currentTime.Sub(timeStamp) >= 10*time.Second {
				delete(handler.scoutingMap[team], name)
			}
		}
	}

	var response RequestCurrentScoutingResponseT
	for name, _ := range handler.scoutingMap[teamNumber] {
		if name != collectedBy {
			response.CollectedBy = append(response.CollectedBy, &request_current_scouting_response.CollectedByT{
				Name: name,
			})
		}
	}

	builder := flatbuffers.NewBuilder(10)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type submitNoteScoutingHandler struct {
	db Database
}

func (handler submitNoteScoutingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "SubmitNotes", submit_notes.GetRootAsSubmitNotes)
	if !success {
		return
	}

	err = handler.db.AddNotes(db.NotesData{
		TeamNumber:     string(request.Team()),
		Notes:          string(request.Notes()),
		GoodDriving:    bool(request.GoodDriving()),
		BadDriving:     bool(request.BadDriving()),
		SolidPlacing:   bool(request.SolidPlacing()),
		SketchyPlacing: bool(request.SketchyPlacing()),
		GoodDefense:    bool(request.GoodDefense()),
		BadDefense:     bool(request.BadDefense()),
		EasilyDefended: bool(request.EasilyDefended()),
		NoShow:         bool(request.NoShow()),
		MatchNumber:    request.MatchNumber(),
		SetNumber:      request.SetNumber(),
		CompLevel:      string(request.CompLevel()),
	})
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to insert notes: %v", err))
		return
	}

	var response SubmitNotesResponseT
	builder := flatbuffers.NewBuilder(10)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type submitPitImageScoutingHandler struct {
	db Database
}

func (handler submitPitImageScoutingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "SubmitPitImage", submit_pit_image.GetRootAsSubmitPitImage)
	if !success {
		return
	}

	err = handler.db.AddPitImage(db.PitImage{
		TeamNumber: string(request.TeamNumber()),
		CheckSum:   db.ComputeSha256FromByteArray(request.ImageDataBytes()),
		ImagePath:  string(request.ImagePath()),
		ImageData:  request.ImageDataBytes(),
	})
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to insert notes: %v", err))
		return
	}

	var response SubmitPitImageResponseT
	builder := flatbuffers.NewBuilder(10)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

func ConvertActionsToStat2024(submit2024Actions *submit_2024_actions.Submit2024Actions) (db.Stats2024, error) {
	overall_time := int64(0)
	cycles := int64(0)
	picked_up := false
	lastPlacedTime := int64(0)
	stat := db.Stats2024{
		CompType: string(submit2024Actions.CompType()), TeamNumber: string(submit2024Actions.TeamNumber()),
		MatchNumber: submit2024Actions.MatchNumber(), SetNumber: submit2024Actions.SetNumber(), CompLevel: string(submit2024Actions.CompLevel()),
		StartingQuadrant: 0, SpeakerAuto: 0, AmpAuto: 0, NotesDroppedAuto: 0, MobilityAuto: false,
		Speaker: 0, Amp: 0, SpeakerAmplified: 0, NotesDropped: 0, Shuttled: 0, OutOfField: 0, Penalties: 0,
		TrapNote: false, Spotlight: false, AvgCycle: 0, Park: false, OnStage: false, Harmony: false, RobotDied: false, NoShow: false, CollectedBy: "",
	}
	// Loop over all actions.
	for i := 0; i < submit2024Actions.ActionsListLength(); i++ {
		var action submit_2024_actions.Action
		if !submit2024Actions.ActionsList(&action, i) {
			return db.Stats2024{}, errors.New(fmt.Sprintf("Failed to parse submit_2024_actions.Action"))
		}
		actionTable := new(flatbuffers.Table)
		action_type := action.ActionTakenType()
		if !action.ActionTaken(actionTable) {
			return db.Stats2024{}, errors.New(fmt.Sprint("Failed to parse sub-action or sub-action was missing"))
		}
		if action_type == submit_2024_actions.ActionTypeStartMatchAction {
			var startMatchAction submit_2024_actions.StartMatchAction
			startMatchAction.Init(actionTable.Bytes, actionTable.Pos)
			stat.StartingQuadrant = startMatchAction.Position()
		} else if action_type == submit_2024_actions.ActionTypeMobilityAction {
			var mobilityAction submit_2024_actions.MobilityAction
			mobilityAction.Init(actionTable.Bytes, actionTable.Pos)
			if mobilityAction.Mobility() {
				stat.MobilityAuto = true
			}
		} else if action_type == submit_2024_actions.ActionTypePenaltyAction {
			var penaltyAction submit_2024_actions.PenaltyAction
			penaltyAction.Init(actionTable.Bytes, actionTable.Pos)
			stat.Penalties += penaltyAction.Penalties()

		} else if action_type == submit_2024_actions.ActionTypeRobotDeathAction {
			var robotDeathAction submit_2024_actions.RobotDeathAction
			robotDeathAction.Init(actionTable.Bytes, actionTable.Pos)
			stat.RobotDied = true

		} else if action_type == submit_2024_actions.ActionTypeNoShowAction {
			var NoShowAction submit_2024_actions.NoShowAction
			NoShowAction.Init(actionTable.Bytes, actionTable.Pos)
			stat.NoShow = true

		} else if action_type == submit_2024_actions.ActionTypePickupNoteAction {
			var pick_up_action submit_2024_actions.PickupNoteAction
			pick_up_action.Init(actionTable.Bytes, actionTable.Pos)
			picked_up = true
		} else if action_type == submit_2024_actions.ActionTypePlaceNoteAction {
			var place_action submit_2024_actions.PlaceNoteAction
			place_action.Init(actionTable.Bytes, actionTable.Pos)
			if !picked_up {
				return db.Stats2024{}, errors.New(fmt.Sprintf("Got PlaceNoteAction without corresponding PickupObjectAction"))
			}
			score_type := place_action.ScoreType()
			auto := place_action.Auto()
			count_in_cycle := true
			if score_type == submit_2024_actions.ScoreTypekAMP && auto {
				stat.AmpAuto += 1
			} else if score_type == submit_2024_actions.ScoreTypekAMP && !auto {
				stat.Amp += 1
			} else if score_type == submit_2024_actions.ScoreTypekSPEAKER && !auto {
				stat.Speaker += 1
			} else if score_type == submit_2024_actions.ScoreTypekSPEAKER && auto {
				stat.SpeakerAuto += 1
			} else if score_type == submit_2024_actions.ScoreTypekSPEAKER_AMPLIFIED && !auto {
				stat.SpeakerAmplified += 1
			} else if score_type == submit_2024_actions.ScoreTypekDROPPED && auto {
				stat.NotesDroppedAuto += 1
				count_in_cycle = false
			} else if score_type == submit_2024_actions.ScoreTypekDROPPED && !auto {
				stat.NotesDropped += 1
				count_in_cycle = false
			} else if score_type == submit_2024_actions.ScoreTypekSHUTTLED {
				stat.Shuttled += 1
				count_in_cycle = false
			} else if score_type == submit_2024_actions.ScoreTypekOUT_OF_FIELD {
				stat.OutOfField += 1
				count_in_cycle = false
			} else {
				return db.Stats2024{}, errors.New(fmt.Sprintf("Got unknown ObjectType/ScoreLevel/Auto combination"))
			}
			picked_up = false
			if count_in_cycle {
				// Assuming dropped, shuttled, and out of field
				// notes are not counted in total cycle time.
				if lastPlacedTime != int64(0) {
					// If this is not the first time we place,
					// start counting cycle time. We define cycle
					// time as the time between placements.
					overall_time += int64(action.Timestamp()) - lastPlacedTime
				}
				cycles += 1
				lastPlacedTime = int64(action.Timestamp())
			}
		} else if action_type == submit_2024_actions.ActionTypeEndMatchAction {
			var endMatchAction submit_2024_actions.EndMatchAction
			endMatchAction.Init(actionTable.Bytes, actionTable.Pos)
			if endMatchAction.StageType() == submit_2024_actions.StageTypekON_STAGE {
				stat.OnStage = true
			} else if endMatchAction.StageType() == submit_2024_actions.StageTypekPARK {
				stat.Park = true
			} else if endMatchAction.StageType() == submit_2024_actions.StageTypekHARMONY {
				stat.Harmony = true
			}
			stat.TrapNote = endMatchAction.TrapNote()
			stat.Spotlight = endMatchAction.Spotlight()
		}
	}
	if cycles != 0 {
		stat.AvgCycle = overall_time / cycles
	} else {
		stat.AvgCycle = 0
	}
	return stat, nil
}

// Handles a Request2024DataScouting request.
type request2024DataScoutingHandler struct {
	db Database
}

func (handler request2024DataScoutingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	_, success := parseRequest(w, requestBytes, "Request2024DataScouting", request_2024_data_scouting.GetRootAsRequest2024DataScouting)
	if !success {
		return
	}

	stats, err := handler.db.ReturnStats2024()
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to query database: ", err))
		return
	}

	var response Request2024DataScoutingResponseT
	for _, stat := range stats {
		response.StatsList = append(response.StatsList, &request_2024_data_scouting_response.Stats2024T{
			TeamNumber:       stat.TeamNumber,
			MatchNumber:      stat.MatchNumber,
			SetNumber:        stat.SetNumber,
			CompLevel:        stat.CompLevel,
			StartingQuadrant: stat.StartingQuadrant,
			SpeakerAuto:      stat.SpeakerAuto,
			AmpAuto:          stat.AmpAuto,
			NotesDroppedAuto: stat.NotesDroppedAuto,
			MobilityAuto:     stat.MobilityAuto,
			Speaker:          stat.Speaker,
			Amp:              stat.Amp,
			SpeakerAmplified: stat.SpeakerAmplified,
			NotesDropped:     stat.NotesDropped,
			Shuttled:         stat.Shuttled,
			OutOfField:       stat.OutOfField,
			Penalties:        stat.Penalties,
			TrapNote:         stat.TrapNote,
			Spotlight:        stat.Spotlight,
			AvgCycle:         stat.AvgCycle,
			Park:             stat.Park,
			OnStage:          stat.OnStage,
			Harmony:          stat.Harmony,
			RobotDied:        stat.RobotDied,
			NoShow:           stat.NoShow,
			CollectedBy:      stat.CollectedBy,
			CompType:         stat.CompType,
		})
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type requestAllPitImagesHandler struct {
	db Database
}

func (handler requestAllPitImagesHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	_, success := parseRequest(w, requestBytes, "RequestAllPitImages", request_all_pit_images.GetRootAsRequestAllPitImages)
	if !success {
		return
	}

	images, err := handler.db.ReturnPitImages()
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to get pit images: %v", err))
		return
	}

	var response RequestAllPitImagesResponseT
	for _, data := range images {
		response.PitImageList = append(response.PitImageList, &request_all_pit_images_response.PitImageT{
			TeamNumber: data.TeamNumber,
			ImagePath:  data.ImagePath,
			CheckSum:   data.CheckSum,
		})
	}

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type requestPitImagesHandler struct {
	db Database
}

func (handler requestPitImagesHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "RequestPitImages", request_pit_images.GetRootAsRequestPitImages)
	if !success {
		return
	}

	images, err := handler.db.QueryPitImages(string(request.TeamNumber()))
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to query pit images: %v", err))
		return
	}

	var response RequestPitImagesResponseT
	for _, data := range images {
		response.PitImageList = append(response.PitImageList, &request_pit_images_response.PitImageT{
			TeamNumber: data.TeamNumber,
			ImagePath:  data.ImagePath,
			CheckSum:   data.CheckSum,
		})
	}

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type requestNotesForTeamHandler struct {
	db Database
}

func (handler requestNotesForTeamHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "RequestNotesForTeam", request_notes_for_team.GetRootAsRequestNotesForTeam)
	if !success {
		return
	}

	notes, err := handler.db.QueryNotes(string(request.Team()))
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to query notes: %v", err))
		return
	}

	var response RequestNotesForTeamResponseT
	for _, data := range notes {
		response.Notes = append(response.Notes, &request_notes_for_team_response.NoteT{data})
	}

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type requestShiftScheduleHandler struct {
	db Database
}

func (handler requestShiftScheduleHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	_, success := parseRequest(w, requestBytes, "RequestShiftSchedule", request_shift_schedule.GetRootAsRequestShiftSchedule)
	if !success {
		return
	}

	shiftData, err := handler.db.ReturnAllShifts()
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to query shift schedule: %v", err))
		return
	}

	var response RequestShiftScheduleResponseT
	for _, shifts := range shiftData {
		response.ShiftSchedule = append(response.ShiftSchedule, &request_shift_schedule_response.MatchAssignmentT{
			MatchNumber: shifts.MatchNumber,
			R1Scouter:   shifts.R1scouter,
			R2Scouter:   shifts.R2scouter,
			R3Scouter:   shifts.R3scouter,
			B1Scouter:   shifts.B1scouter,
			B2Scouter:   shifts.B2scouter,
			B3Scouter:   shifts.B3scouter,
		})
	}

	builder := flatbuffers.NewBuilder(1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type submitShiftScheduleHandler struct {
	db Database
}

func (handler submitShiftScheduleHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	// Get the username of the person submitting the data.
	username := parseUsername(req)

	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest[SubmitShiftSchedule](w, requestBytes, "SubmitShiftSchedule", submit_shift_schedule.GetRootAsSubmitShiftSchedule)
	if !success {
		return
	}

	log.Println("Got shift schedule from", username)
	shift_schedule_length := request.ShiftScheduleLength()
	for i := 0; i < shift_schedule_length; i++ {
		var match_assignment submit_shift_schedule.MatchAssignment
		request.ShiftSchedule(&match_assignment, i)
		current_shift := db.Shift{
			MatchNumber: match_assignment.MatchNumber(),
			R1scouter:   string(match_assignment.R1Scouter()),
			R2scouter:   string(match_assignment.R2Scouter()),
			R3scouter:   string(match_assignment.R3Scouter()),
			B1scouter:   string(match_assignment.B1Scouter()),
			B2scouter:   string(match_assignment.B2Scouter()),
			B3scouter:   string(match_assignment.B3Scouter()),
		}
		err = handler.db.AddToShift(current_shift)
		if err != nil {
			respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to submit shift schedule: ", err))
			return
		}
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&SubmitShiftScheduleResponseT{}).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type SubmitDriverRankingHandler struct {
	db Database
}

func (handler SubmitDriverRankingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "SubmitDriverRanking", submit_driver_ranking.GetRootAsSubmitDriverRanking)
	if !success {
		return
	}

	err = handler.db.AddDriverRanking(db.DriverRankingData{
		MatchNumber: request.MatchNumber(),
		Rank1:       string(request.Rank1()),
		Rank2:       string(request.Rank2()),
		Rank3:       string(request.Rank3()),
	})

	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to insert driver ranking: %v", err))
		return
	}

	var response SubmitDriverRankingResponseT
	builder := flatbuffers.NewBuilder(10)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type requestAllNotesHandler struct {
	db Database
}

func (handler requestAllNotesHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	_, success := parseRequest(w, requestBytes, "RequestAllNotes", request_all_notes.GetRootAsRequestAllNotes)
	if !success {
		return
	}

	notes, err := handler.db.ReturnAllNotes()
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to query database: ", err))
		return
	}

	var response RequestAllNotesResponseT
	for _, note := range notes {
		response.NoteList = append(response.NoteList, &request_all_notes_response.NoteT{
			Team:           note.TeamNumber,
			Notes:          note.Notes,
			GoodDriving:    note.GoodDriving,
			BadDriving:     note.BadDriving,
			SolidPlacing:   note.SolidPlacing,
			SketchyPlacing: note.SketchyPlacing,
			GoodDefense:    note.GoodDefense,
			BadDefense:     note.BadDefense,
			EasilyDefended: note.EasilyDefended,
			NoShow:         note.NoShow,
			MatchNumber:    note.MatchNumber,
			CompLevel:      note.CompLevel,
			SetNumber:      note.SetNumber,
		})
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type requestAllDriverRankingsHandler struct {
	db Database
}

func (handler requestAllDriverRankingsHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	_, success := parseRequest(w, requestBytes, "RequestAllDriverRankings", request_all_driver_rankings.GetRootAsRequestAllDriverRankings)
	if !success {
		return
	}

	rankings, err := handler.db.ReturnAllDriverRankings()
	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to query database: ", err))
		return
	}

	var response RequestAllDriverRankingsResponseT
	for _, ranking := range rankings {
		response.DriverRankingList = append(response.DriverRankingList, &request_all_driver_rankings_response.RankingT{
			MatchNumber: ranking.MatchNumber,
			Rank1:       ranking.Rank1,
			Rank2:       ranking.Rank2,
			Rank3:       ranking.Rank3,
		})
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

type submit2024ActionsHandler struct {
	db Database
}

func (handler submit2024ActionsHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	// Get the username of the person submitting the data.
	username := parseUsername(req)

	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		log.Println("Failed to receive submission request from", username)
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "Submit2024Actions", submit_2024_actions.GetRootAsSubmit2024Actions)
	if !success {
		log.Println("Failed to parse submission request from", username)
		return
	}

	log.Println("Got actions for match", request.MatchNumber(), "team", string(request.TeamNumber()), "type", string(request.CompType()), "from", username)

	for i := 0; i < request.ActionsListLength(); i++ {

		var action Action2024
		request.ActionsList(&action, i)

		dbAction := db.Action{
			CompType:    string(request.CompType()),
			TeamNumber:  string(request.TeamNumber()),
			MatchNumber: request.MatchNumber(),
			SetNumber:   request.SetNumber(),
			CompLevel:   string(request.CompLevel()),
			//TODO: Serialize CompletedAction
			CompletedAction: []byte{},
			Timestamp:       action.Timestamp(),
			CollectedBy:     username,
		}

		// Do some error checking.
		if action.Timestamp() < 0 {
			log.Println("Got action with invalid timestamp (", action.Timestamp(), ") from", username)
			respondWithError(w, http.StatusBadRequest, fmt.Sprint(
				"Invalid timestamp field value of ", action.Timestamp()))
			return
		}

		err = handler.db.AddAction(dbAction)
		if err != nil {
			log.Println("Failed to add action from", username, "to the database:", err)
			respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to add action to database: ", err))
			return
		}
	}

	stats, err := ConvertActionsToStat2024(request)
	if err != nil {
		log.Println("Failed to add action from", username, "to the database:", err)
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to convert actions to stats: ", err))
		return
	}

	stats.CollectedBy = username

	err = handler.db.AddToStats2024(stats)
	if err != nil {
		log.Println("Failed to submit stats from", username, "to the database:", err)
		respondWithError(w, http.StatusInternalServerError, fmt.Sprint("Failed to submit stats2024: ", stats, ": ", err))
		return
	}

	builder := flatbuffers.NewBuilder(50 * 1024)
	builder.Finish((&Submit2024ActionsResponseT{}).Pack(builder))
	w.Write(builder.FinishedBytes())

	log.Println("Successfully added stats from", username)
}

type Delete2024DataScoutingHandler struct {
	db Database
}

func (handler Delete2024DataScoutingHandler) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	requestBytes, err := io.ReadAll(req.Body)
	if err != nil {
		respondWithError(w, http.StatusBadRequest, fmt.Sprint("Failed to read request bytes:", err))
		return
	}

	request, success := parseRequest(w, requestBytes, "Delete2024DataScouting", delete_2024_data_scouting.GetRootAsDelete2024DataScouting)
	if !success {
		return
	}

	err = handler.db.DeleteFromStats2024(
		string(request.CompLevel()),
		request.MatchNumber(),
		request.SetNumber(),
		string(request.TeamNumber()))

	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to delete from stats2024: %v", err))
		return
	}

	err = handler.db.DeleteFromActions(
		string(request.CompLevel()),
		request.MatchNumber(),
		request.SetNumber(),
		string(request.TeamNumber()))

	if err != nil {
		respondWithError(w, http.StatusInternalServerError, fmt.Sprintf("Failed to delete from actions: %v", err))
		return
	}

	var response Delete2024DataScoutingResponseT
	builder := flatbuffers.NewBuilder(10)
	builder.Finish((&response).Pack(builder))
	w.Write(builder.FinishedBytes())
}

func HandleRequests(db Database, scoutingServer server.ScoutingServer, clock Clock) {
	scoutingServer.HandleFunc("/requests", unknown)
	scoutingServer.Handle("/requests/request/all_matches", requestAllMatchesHandler{db})
	scoutingServer.Handle("/requests/request/all_notes", requestAllNotesHandler{db})
	scoutingServer.Handle("/requests/request/all_driver_rankings", requestAllDriverRankingsHandler{db})
	scoutingServer.Handle("/requests/request/2024_data_scouting", request2024DataScoutingHandler{db})
	scoutingServer.Handle("/requests/submit/submit_notes", submitNoteScoutingHandler{db})
	scoutingServer.Handle("/requests/submit/submit_pit_image", submitPitImageScoutingHandler{db})
	scoutingServer.Handle("/requests/request/pit_images", requestPitImagesHandler{db})
	scoutingServer.Handle("/requests/request/all_pit_images", requestAllPitImagesHandler{db})
	scoutingServer.Handle("/requests/request/current_scouting", requestCurrentScoutingHandler{make(map[string]map[string]time.Time), db, clock})
	scoutingServer.Handle("/requests/request/notes_for_team", requestNotesForTeamHandler{db})
	scoutingServer.Handle("/requests/submit/shift_schedule", submitShiftScheduleHandler{db})
	scoutingServer.Handle("/requests/request/shift_schedule", requestShiftScheduleHandler{db})
	scoutingServer.Handle("/requests/submit/submit_driver_ranking", SubmitDriverRankingHandler{db})
	scoutingServer.Handle("/requests/submit/submit_2024_actions", submit2024ActionsHandler{db})
	scoutingServer.Handle("/requests/delete/delete_2024_data_scouting", Delete2024DataScoutingHandler{db})
}
