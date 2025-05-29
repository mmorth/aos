import {Injectable} from '@angular/core';
import {Builder, ByteBuffer} from 'flatbuffers';
import {ErrorResponse} from '@aos/scouting/webserver/requests/messages/error_response_ts_fbs/scouting/webserver/requests';
import {RequestAllMatches} from '@aos/scouting/webserver/requests/messages/request_all_matches_ts_fbs/scouting/webserver/requests';
import {
  Match,
  RequestAllMatchesResponse,
} from '@aos/scouting/webserver/requests/messages/request_all_matches_response_ts_fbs/scouting/webserver/requests';
import {db, MatchListData} from './db';

const MATCH_TYPE_ORDERING = ['qm', 'ef', 'qf', 'sf', 'f'];

@Injectable({providedIn: 'root'})
export class MatchListRequestor {
  async fetchMatchList(): Promise<Match[]> {
    const builder = new Builder();
    RequestAllMatches.startRequestAllMatches(builder);
    builder.finish(RequestAllMatches.endRequestAllMatches(builder));
    const buffer = builder.asUint8Array();
    const res = await fetch('/requests/request/all_matches', {
      method: 'POST',
      body: buffer,
    });
    if (res.ok) {
      const resBuffer = await res.arrayBuffer();
      const u8Buffer = new Uint8Array(resBuffer);
      // Cache the response.
      await db.matchListData.put({id: 1, data: u8Buffer});
      return this.parseMatchList(u8Buffer);
    } else {
      const cachedResult = await db.matchListData.where({id: 1}).toArray();
      if (cachedResult && cachedResult.length == 1) {
        const u8Buffer = cachedResult[0].data;
        return this.parseMatchList(u8Buffer);
      }
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(fbBuffer);
      const errorMessage = parsedResponse.errorMessage();
      throw `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
    }
  }
  parseMatchList(u8Buffer: Uint8Array): Match[] {
    const fbBuffer = new ByteBuffer(u8Buffer);
    const parsedResponse =
      RequestAllMatchesResponse.getRootAsRequestAllMatchesResponse(fbBuffer);
    // Convert the flatbuffer list into an array. That's more useful.
    const matchList = [];
    for (let i = 0; i < parsedResponse.matchListLength(); i++) {
      matchList.push(parsedResponse.matchList(i));
    }
    // Sort the list so it is in chronological order.
    matchList.sort((a, b) => {
      // First sort by match type. E.g. finals are last.
      const aMatchTypeIndex = MATCH_TYPE_ORDERING.indexOf(a.compLevel());
      const bMatchTypeIndex = MATCH_TYPE_ORDERING.indexOf(b.compLevel());
      if (aMatchTypeIndex < bMatchTypeIndex) {
        return -1;
      }
      if (aMatchTypeIndex > bMatchTypeIndex) {
        return 1;
      }
      // Then sort by match number. E.g. in semi finals, all match 1 rounds
      // are done first. Then come match 2 rounds. And then, if necessary,
      // the match 3 rounds.
      const aMatchNumber = a.matchNumber();
      const bMatchNumber = b.matchNumber();
      if (aMatchNumber < bMatchNumber) {
        return -1;
      }
      if (aMatchNumber > bMatchNumber) {
        return 1;
      }
      // Lastly, sort by set number. I.e. Semi Final 1 Match 1 happens first.
      // Then comes Semi Final 2 Match 1. Then comes Semi Final 1 Match 2. Then
      // Semi Final 2 Match 2.
      const aSetNumber = a.setNumber();
      const bSetNumber = b.setNumber();
      if (aSetNumber < bSetNumber) {
        return -1;
      }
      if (aSetNumber > bSetNumber) {
        return 1;
      }
      return 0;
    });
    return matchList;
  }
}
