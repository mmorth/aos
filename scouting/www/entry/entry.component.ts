import {
  Component,
  ElementRef,
  EventEmitter,
  Input,
  OnInit,
  Output,
  ViewChild,
} from '@angular/core';
import {FormsModule} from '@angular/forms';
import {Builder, ByteBuffer} from 'flatbuffers';
import {ErrorResponse} from '@aos/scouting/webserver/requests/messages/error_response_ts_fbs/scouting/webserver/requests';
import {
  StartMatchAction,
  StartMatchActionT,
  NoShowActionT,
  NoShowAction,
  ScoreType,
  StageType,
  Submit2024Actions,
  MobilityAction,
  MobilityActionT,
  PenaltyAction,
  PenaltyActionT,
  PickupNoteAction,
  PickupNoteActionT,
  PlaceNoteAction,
  PlaceNoteActionT,
  RobotDeathAction,
  RobotDeathActionT,
  EndMatchAction,
  EndMatchActionT,
  ActionType,
  Action,
  ActionT,
} from '@aos/scouting/webserver/requests/messages/submit_2024_actions_ts_fbs/scouting/webserver/requests';
import {Match} from '@aos/scouting/webserver/requests/messages/request_all_matches_response_ts_fbs/scouting/webserver/requests';
import {MatchListRequestor, ActionsSubmitter} from '@aos/scouting/www/rpc';
import {RequestCurrentScouting} from '@aos/scouting/webserver/requests/messages/request_current_scouting_ts_fbs/scouting/webserver/requests';
import {RequestCurrentScoutingResponse} from '@aos/scouting/webserver/requests/messages/request_current_scouting_response_ts_fbs/scouting/webserver/requests';
import {ActionHelper, ConcreteAction} from './action_helper';
import * as pako from 'pako';

type Section =
  | 'Team Selection'
  | 'Init'
  | 'Pickup'
  | 'Place'
  | 'Endgame'
  | 'Dead'
  | 'Review and Submit'
  | 'QR Code'
  | 'Success';

type CompType = 'PreScouting' | 'Practice' | 'Regular';

// TODO(phil): Deduplicate with match_list.component.ts.
const COMP_LEVELS = ['qm', 'ef', 'qf', 'sf', 'f'] as const;
export type CompLevel = typeof COMP_LEVELS[number];

// TODO(phil): Deduplicate with match_list.component.ts.
const COMP_LEVEL_LABELS: Record<CompLevel, string> = {
  qm: 'Qualifications',
  ef: 'Eighth Finals',
  qf: 'Quarter Finals',
  sf: 'Semi Finals',
  f: 'Finals',
};

// The maximum number of bytes per QR code. The user can adjust this value to
// make the QR code contain less information, but easier to scan.
const QR_CODE_PIECE_SIZES = [150, 300, 450, 600, 750, 900];

// The default index into QR_CODE_PIECE_SIZES.
const DEFAULT_QR_CODE_PIECE_SIZE_INDEX = QR_CODE_PIECE_SIZES.indexOf(450);

// The actions that are purely used for tracking state. They don't actually
// have any permanent meaning and will not be saved in the database.
const STATE_ACTIONS: ActionType[] = [
  ActionType.EndAutoPhaseAction,
  ActionType.EndTeleopPhaseAction,
];

@Component({
  selector: 'app-entry',
  templateUrl: './entry.ng.html',
  styleUrls: ['../app/common.css', './entry.component.css'],
})
export class EntryComponent implements OnInit {
  // Re-export the type here so that we can use it in the `[value]` attribute
  // of radio buttons.
  readonly COMP_LEVELS = COMP_LEVELS;
  readonly COMP_LEVEL_LABELS = COMP_LEVEL_LABELS;
  readonly QR_CODE_PIECE_SIZES = QR_CODE_PIECE_SIZES;
  readonly ScoreType = ScoreType;
  readonly StageType = StageType;
  readonly ActionT = ActionT;
  readonly ActionType = ActionType;
  readonly NoShowActionT = NoShowActionT;
  readonly StartMatchActionT = StartMatchActionT;
  readonly MobilityActionT = MobilityActionT;
  readonly PickupNoteActionT = PickupNoteActionT;
  readonly PlaceNoteActionT = PlaceNoteActionT;
  readonly RobotDeathActionT = RobotDeathActionT;
  readonly PenaltyActionT = PenaltyActionT;
  readonly EndMatchActionT = EndMatchActionT;

  section: Section = 'Team Selection';
  @Input() matchNumber: number = 1;
  @Input() teamNumber: string = '1';
  @Input() setNumber: number = 1;
  @Input() compLevel: CompLevel = 'qm';
  @Input() compType: CompType = 'Regular';
  @Input() skipTeamSelection = false;

  @ViewChild('header') header: ElementRef;

  matchList: Match[] = [];

  actionHelper: ActionHelper;
  actionList: ActionT[] = [];
  progressMessage: string = '';
  errorMessage: string = '';
  autoPhase: boolean = true;
  mobilityCompleted: boolean = false;
  // TODO(phil): Come up with a better name here.
  selectedValue = 0;
  endGameAction: StageType = StageType.kMISSING;
  noteIsTrapped: boolean = false;
  endGameSpotlight: boolean = false;

  nextTeamNumber = '';

  matchStartTimestamp: number = 0;
  penalties: number = 0;

  teamSelectionIsValid = false;
  searchForDuplicateScouting: boolean = false;
  duplicateSearchTimer: ReturnType<typeof setTimeout> | null = null;

  // When the user chooses to generate QR codes, we convert the flatbuffer into
  // a long string. Since we frequently have more data than we can display in a
  // single QR code, we break the data into multiple QR codes. The data for
  // each QR code ("pieces") is stored in the `qrCodeValuePieces` list below.
  // The `qrCodeValueIndex` keeps track of which QR code we're currently
  // displaying.
  qrCodeValuePieceSize = QR_CODE_PIECE_SIZES[DEFAULT_QR_CODE_PIECE_SIZE_INDEX];
  qrCodeValuePieces: string[] = [];
  qrCodeValueIndex: number = 0;

  constructor(
    private readonly matchListRequestor: MatchListRequestor,
    private readonly actionsSubmitter: ActionsSubmitter
  ) {}

  ngOnInit() {
    this.actionHelper = new ActionHelper(
      (actionType: ActionType, action: ConcreteAction) => {
        this.addAction(actionType, action);
      }
    );

    // When the user navigated from the match list, we can skip the team
    // selection. I.e. we trust that the user clicked the correct button.
    this.section = this.skipTeamSelection ? 'Init' : 'Team Selection';
    this.fetchMatchList();
  }

  ngOnDestroy() {
    clearInterval(this.duplicateSearchTimer);
  }

  goToNextTeam() {
    this.ngOnInit();
    this.ngOnDestroy();
    this.teamNumber = this.nextTeamNumber;
    this.nextTeamNumber = '';
  }

  async fetchMatchList() {
    this.progressMessage = 'Fetching match list. Please be patient.';
    this.errorMessage = '';

    try {
      this.matchList = await this.matchListRequestor.fetchMatchList();
      this.progressMessage = 'Successfully fetched match list.';
    } catch (e) {
      this.errorMessage = e;
      this.progressMessage = '';
    }
  }

  async findOthersScoutingThisRobot() {
    const builder = new Builder();
    const teamNumber = builder.createString(this.teamNumber);

    builder.finish(
      RequestCurrentScouting.createRequestCurrentScouting(builder, teamNumber)
    );

    const buffer = builder.asUint8Array();
    const res = await fetch('/requests/request/current_scouting', {
      method: 'POST',
      body: buffer,
    });

    if (!res.ok) {
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(fbBuffer);
      const errorMessage = parsedResponse.errorMessage();
      this.errorMessage = `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
    } else {
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse =
        RequestCurrentScoutingResponse.getRootAsRequestCurrentScoutingResponse(
          fbBuffer
        );
      const collectedBy = [];
      for (let i = 0; i < parsedResponse.collectedByLength(); i++) {
        collectedBy.push(parsedResponse.collectedBy(i).name());
      }
      this.duplicateSearchTimer = setInterval(() => {
        if (this.searchForDuplicateScouting && collectedBy.length != 0) {
          if (
            confirm(
              'This team is currently being scouted by ' +
                collectedBy +
                '. Would you like to receive more alerts?'
            ) != true
          ) {
            this.searchForDuplicateScouting = false;
          }
        }
      }, 10000);
    }
  }

  // This gets called when the user changes something on the Init screen.
  // It makes sure that the user can't click "Next" until the information is
  // valid, or this is for pre-scouting or practice matches.
  updateTeamSelectionValidity(): void {
    this.teamSelectionIsValid =
      this.compType != 'Regular' || this.matchIsInMatchList();
  }

  matchIsInMatchList(): boolean {
    // If the user deletes the content of the teamNumber field, the value here
    // is undefined. Guard against that.
    if (this.teamNumber == null) {
      return false;
    }

    for (const match of this.matchList) {
      if (
        this.matchNumber == match.matchNumber() &&
        this.setNumber == match.setNumber() &&
        this.compLevel == match.compLevel() &&
        (this.teamNumber === match.r1() ||
          this.teamNumber === match.r2() ||
          this.teamNumber === match.r3() ||
          this.teamNumber === match.b1() ||
          this.teamNumber === match.b2() ||
          this.teamNumber === match.b3())
      ) {
        return true;
      }
    }
    return false;
  }

  addPenalty(): void {
    this.penalties += 1;
  }

  removePenalty(): void {
    if (this.penalties > 0) {
      this.penalties -= 1;
    }
  }

  addPenalties(): void {
    this.actionHelper.addPenaltyAction({penalties: this.penalties});
  }

  addAction(actionType: ActionType, action: ConcreteAction): void {
    let timestamp: number = 0;

    if (actionType == ActionType.StartMatchAction) {
      // Unix nanosecond timestamp.
      this.matchStartTimestamp = Date.now() * 1e6;
    } else {
      // Unix nanosecond timestamp relative to match start.
      timestamp = Date.now() * 1e6 - this.matchStartTimestamp;
    }

    if (actionType == ActionType.EndMatchAction) {
      // endMatchAction occurs at the same time as penaltyAction so add to its
      // timestamp to make it unique.
      timestamp += 1;
    }

    if (actionType == ActionType.MobilityAction) {
      this.mobilityCompleted = true;
    }

    this.actionList.push(new ActionT(BigInt(timestamp), actionType, action));
  }

  undoLastAction() {
    if (this.actionList.length > 0) {
      let lastAction = this.actionList.pop();
      switch (lastAction?.actionTakenType) {
        case ActionType.EndAutoPhaseAction:
          this.autoPhase = true;
          this.section = 'Pickup';
          break;
        case ActionType.NoShowAction:
          this.autoPhase = true;
          this.section = 'Init';
          break;
        case ActionType.PickupNoteAction:
          this.section = 'Pickup';
          break;
        case ActionType.EndTeleopPhaseAction:
          this.section = 'Pickup';
          break;
        case ActionType.PlaceNoteAction:
          this.section = 'Place';
          break;
        case ActionType.EndMatchAction:
          this.section = 'Endgame';
          // Also delete the penalty action.
          this.undoLastAction();
          break;
        case ActionType.MobilityAction:
          this.mobilityCompleted = false;
          break;
        case ActionType.StartMatchAction:
          this.section = 'Init';
          break;
        case ActionType.RobotDeathAction:
          // TODO(FILIP): Return user to the screen they
          // clicked dead robot on. Pickup is fine for now but
          // might cause confusion.
          this.section = 'Pickup';
          break;
        default:
          break;
      }
    }
  }

  stringifyScoreType(scoreType: ScoreType): String {
    return ScoreType[scoreType];
  }

  stringifyStageType(stageType: StageType): String {
    return StageType[stageType];
  }

  changeSectionTo(target: Section) {
    // Clear the messages since they won't be relevant in the next section.
    this.errorMessage = '';
    this.progressMessage = '';

    // For the QR code screen, we need to make the value to encode available.
    if (target == 'QR Code') {
      this.updateQrCodeValuePieceSize();
    }

    if (target == 'Init') {
      this.searchForDuplicateScouting = true;
      this.findOthersScoutingThisRobot();
    }

    this.section = target;
  }

  private scrollToTop() {
    this.header.nativeElement.scrollIntoView();
  }

  createActionsBuffer() {
    const builder = new Builder();
    const actionOffsets: number[] = [];

    for (const action of this.actionList) {
      if (STATE_ACTIONS.includes(action.actionTakenType)) {
        // Actions only used for undo purposes are not submitted.
        continue;
      }
      actionOffsets.push(action.pack(builder));
    }
    const teamNumberFb = builder.createString(this.teamNumber);
    const compLevelFb = builder.createString(this.compLevel);
    const compTypeFb = builder.createString(this.compType);

    const actionsVector = Submit2024Actions.createActionsListVector(
      builder,
      actionOffsets
    );
    Submit2024Actions.startSubmit2024Actions(builder);
    Submit2024Actions.addTeamNumber(builder, teamNumberFb);
    Submit2024Actions.addMatchNumber(builder, this.matchNumber);
    Submit2024Actions.addSetNumber(builder, this.setNumber);
    Submit2024Actions.addCompLevel(builder, compLevelFb);
    Submit2024Actions.addActionsList(builder, actionsVector);
    Submit2024Actions.addCompType(builder, compTypeFb);
    builder.finish(Submit2024Actions.endSubmit2024Actions(builder));

    return builder.asUint8Array();
  }

  // Same as createActionsBuffer, but encoded as Base64. It's also split into
  // a number of pieces so that each piece is roughly limited to
  // `qrCodeValuePieceSize` bytes.
  createBase64ActionsBuffers(): string[] {
    const originalBuffer = this.createActionsBuffer();
    const deflatedData = pako.deflate(originalBuffer, {level: 9});

    const pieceSize = this.qrCodeValuePieceSize;
    const fullValue = btoa(String.fromCharCode(...deflatedData));
    const numPieces = Math.ceil(fullValue.length / pieceSize);

    let splitData: string[] = [];
    for (let i = 0; i < numPieces; i++) {
      const splitPiece = fullValue.slice(i * pieceSize, (i + 1) * pieceSize);
      splitData.push(`${i}_${numPieces}_${pieceSize}_${splitPiece}`);
    }
    return splitData;
  }

  setQrCodeValueIndex(index: number) {
    this.qrCodeValueIndex = Math.max(
      0,
      Math.min(index, this.qrCodeValuePieces.length - 1)
    );
  }

  updateQrCodeValuePieceSize() {
    this.qrCodeValuePieces = this.createBase64ActionsBuffers();
    this.qrCodeValueIndex = 0;
  }

  async submit2024Actions() {
    const res = await this.actionsSubmitter.submit(this.createActionsBuffer());

    if (res.ok) {
      // We successfully submitted the data. Report success.
      this.section = 'Success';
      this.actionList = [];

      // Keep track of the position of the last robot, use to figure out what
      // the next robot in the same position is.
      let lastTeamPos = '0';
      for (const match of this.matchList) {
        if (
          this.matchNumber === match.matchNumber() &&
          this.setNumber === match.setNumber() &&
          this.compLevel === match.compLevel()
        ) {
          this.teamNumber = this.teamNumber;
          if (this.teamNumber == match.r1()) {
            lastTeamPos = 'r1';
          } else if (this.teamNumber == match.r2()) {
            lastTeamPos = 'r2';
          } else if (this.teamNumber == match.r3()) {
            lastTeamPos = 'r3';
          } else if (this.teamNumber == match.b1()) {
            lastTeamPos = 'b1';
          } else if (this.teamNumber == match.b2()) {
            lastTeamPos = 'b2';
          } else if (this.teamNumber == match.b3()) {
            lastTeamPos = 'b3';
          } else {
            console.log('Position of scouted team not found.');
          }
          break;
        }
      }
      if (lastTeamPos != '0') {
        this.matchNumber += 1;
        for (const match of this.matchList) {
          if (
            this.matchNumber == match.matchNumber() &&
            this.setNumber == match.setNumber() &&
            this.compLevel == match.compLevel()
          ) {
            if (lastTeamPos == 'r1') {
              this.nextTeamNumber = match.r1();
            } else if (lastTeamPos == 'r2') {
              this.nextTeamNumber = match.r2();
            } else if (lastTeamPos == 'r3') {
              this.nextTeamNumber = match.r3();
            } else if (lastTeamPos == 'b1') {
              this.nextTeamNumber = match.b1();
            } else if (lastTeamPos == 'b2') {
              this.nextTeamNumber = match.b2();
            } else if (lastTeamPos == 'b3') {
              this.nextTeamNumber = match.b3();
            } else {
              console.log('Position of last team not found.');
            }
            break;
          }
        }
      } else {
        console.log('Last team position not found.');
      }
      this.matchList = [];
      this.progressMessage = '';
      this.errorMessage = '';
      this.autoPhase = true;
      this.actionList = [];
      this.mobilityCompleted = false;
      this.compType = 'Regular';
      this.matchStartTimestamp = 0;
      this.selectedValue = 0;
      this.searchForDuplicateScouting = false;
    } else {
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(fbBuffer);

      const errorMessage = parsedResponse.errorMessage();
      this.errorMessage = `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
    }
  }
}
