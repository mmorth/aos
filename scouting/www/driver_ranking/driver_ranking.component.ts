import {Component, OnInit} from '@angular/core';
import {Builder, ByteBuffer} from 'flatbuffers';
import {SubmitDriverRanking} from '@aos/scouting/webserver/requests/messages/submit_driver_ranking_ts_fbs/scouting/webserver/requests';
import {ErrorResponse} from '@aos/scouting/webserver/requests/messages/error_response_ts_fbs/scouting/webserver/requests';

// TeamSelection: Display form to input which
// teams to rank and the match number.
// Data: Display the ranking interface where
// the scout can reorder teams and submit data.
type Section = 'TeamSelection' | 'TeamSelectionAdd' | 'Data';

@Component({
  selector: 'app-driver-ranking',
  templateUrl: './driver_ranking.ng.html',
  styleUrls: ['../app/common.css', './driver_ranking.component.css'],
})
export class DriverRankingComponent {
  section: Section = 'TeamSelection';

  // Stores the team keys and rank (order of the array).
  team_ranking: string[] = ['971', '972', '973'];
  added_teams: string[] = ['974', '975', '976'];

  match_number: number = 1;

  errorMessage = '';

  setTeamNumbers() {
    this.section = 'Data';
  }

  addTeamNumbers() {
    for (let i = 0; i < this.added_teams.length; i++) {
      this.team_ranking.push(this.added_teams[i]);
    }
    this.added_teams.splice(0, this.added_teams.length);
    this.added_teams = ['974', '975', '976'];
    this.section = 'Data';
  }

  rankUp(index: number) {
    if (index > 0) {
      this.changeRank(index, index - 1);
    }
  }

  rankDown(index: number) {
    if (this.team_ranking.length == 6) {
      if (index < 5) {
        this.changeRank(index, index + 1);
      }
    } else {
      if (index < 2) {
        this.changeRank(index, index + 1);
      }
    }
  }

  // Change the rank of a team in team_ranking.
  // Move the the team at index 'fromIndex'
  // to the index 'toIndex'.
  // Ex. Moving the rank 2 (index 1) team to rank1 (index 0)
  // would be changeRank(1, 0)

  changeRank(fromIndex: number, toIndex: number) {
    var element = this.team_ranking[fromIndex];
    this.team_ranking.splice(fromIndex, 1);
    this.team_ranking.splice(toIndex, 0, element);
  }

  editTeams() {
    this.section = 'TeamSelection';
  }

  addTeams() {
    this.section = 'TeamSelectionAdd';
  }

  async submitData() {
    const builder = new Builder();
    if (this.team_ranking.length == 3) {
      const teamRanking1 = builder.createString(this.team_ranking[0]);
      const teamRanking2 = builder.createString(this.team_ranking[1]);
      const teamRanking3 = builder.createString(this.team_ranking[2]);
      builder.finish(
        SubmitDriverRanking.createSubmitDriverRanking(
          builder,
          this.match_number,
          teamRanking1,
          teamRanking2,
          teamRanking3
        )
      );
    } else {
      const teamRanking1 = builder.createString(this.team_ranking[0]);
      const teamRanking2 = builder.createString(this.team_ranking[1]);
      const teamRanking3 = builder.createString(this.team_ranking[2]);
      const teamRanking4 = builder.createString(this.team_ranking[3]);
      const teamRanking5 = builder.createString(this.team_ranking[4]);
      const teamRanking6 = builder.createString(this.team_ranking[5]);
      // Submits the ranking 4 times to prevent data loss
      // since driver ranking compares three teams at a time.
      builder.finish(
        SubmitDriverRanking.createSubmitDriverRanking(
          builder,
          this.match_number,
          teamRanking1,
          teamRanking2,
          teamRanking3
        )
      );
      builder.finish(
        SubmitDriverRanking.createSubmitDriverRanking(
          builder,
          this.match_number,
          teamRanking4,
          teamRanking5,
          teamRanking6
        )
      );
      builder.finish(
        SubmitDriverRanking.createSubmitDriverRanking(
          builder,
          this.match_number,
          teamRanking3,
          teamRanking4,
          teamRanking5
        )
      );
      builder.finish(
        SubmitDriverRanking.createSubmitDriverRanking(
          builder,
          this.match_number,
          teamRanking1,
          teamRanking2,
          teamRanking6
        )
      );
    }
    const buffer = builder.asUint8Array();
    const res = await fetch('/requests/submit/submit_driver_ranking', {
      method: 'POST',
      body: buffer,
    });

    if (!res.ok) {
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(fbBuffer);

      const errorMessage = parsedResponse.errorMessage();
      this.errorMessage = `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
      return;
    }

    // Increment the match number.
    this.match_number = this.match_number + 1;

    // Reset Data.
    this.section = 'TeamSelection';
    this.team_ranking = ['971', '972', '973'];
  }
}
