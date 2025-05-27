import {Injectable} from '@angular/core';
import {Builder, ByteBuffer} from 'flatbuffers';
import {ErrorResponse} from '@aos/scouting/webserver/requests/messages/error_response_ts_fbs/scouting/webserver/requests';
import {RequestAllNotes} from '@aos/scouting/webserver/requests/messages/request_all_notes_ts_fbs/scouting/webserver/requests';
import {
  Note,
  RequestAllNotesResponse,
} from '@aos/scouting/webserver/requests/messages/request_all_notes_response_ts_fbs/scouting/webserver/requests';
import {RequestAllDriverRankings} from '@aos/scouting/webserver/requests/messages/request_all_driver_rankings_ts_fbs/scouting/webserver/requests';
import {
  Ranking,
  RequestAllDriverRankingsResponse,
} from '@aos/scouting/webserver/requests/messages/request_all_driver_rankings_response_ts_fbs/scouting/webserver/requests';
import {Request2024DataScouting} from '@aos/scouting/webserver/requests/messages/request_2024_data_scouting_ts_fbs/scouting/webserver/requests';
import {
  PitImage,
  RequestAllPitImagesResponse,
} from '@aos/scouting/webserver/requests/messages/request_all_pit_images_response_ts_fbs/scouting/webserver/requests';
import {RequestAllPitImages} from '@aos/scouting/webserver/requests/messages/request_all_pit_images_ts_fbs/scouting/webserver/requests';
import {
  Stats2024,
  Request2024DataScoutingResponse,
} from '@aos/scouting/webserver/requests/messages/request_2024_data_scouting_response_ts_fbs/scouting/webserver/requests';

@Injectable({providedIn: 'root'})
export class ViewDataRequestor {
  async fetchFromServer(start: Function, end: Function, path: string) {
    const builder = new Builder();
    start(builder);
    builder.finish(end(builder));
    const buffer = builder.asUint8Array();
    const res = await fetch(path, {
      method: 'POST',
      body: buffer,
    });

    const resBuffer = await res.arrayBuffer();
    const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));

    if (!res.ok) {
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(fbBuffer);
      const errorMessage = parsedResponse.errorMessage();
      throw `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
    }

    return fbBuffer;
  }

  // Returns all notes from the database.
  async fetchNoteList(): Promise<Note[]> {
    let fbBuffer = await this.fetchFromServer(
      RequestAllNotes.startRequestAllNotes,
      RequestAllNotes.endRequestAllNotes,
      '/requests/request/all_notes'
    );
    const parsedResponse =
      RequestAllNotesResponse.getRootAsRequestAllNotesResponse(fbBuffer);
    // Convert the flatbuffer list into an array. That's more useful.
    const noteList = [];
    for (let i = 0; i < parsedResponse.noteListLength(); i++) {
      noteList.push(parsedResponse.noteList(i));
    }
    return noteList;
  }
  // Returns all driver ranking entries from the database.
  async fetchDriverRankingList(): Promise<Ranking[]> {
    let fbBuffer = await this.fetchFromServer(
      RequestAllDriverRankings.startRequestAllDriverRankings,
      RequestAllDriverRankings.endRequestAllDriverRankings,
      '/requests/request/all_driver_rankings'
    );

    const parsedResponse =
      RequestAllDriverRankingsResponse.getRootAsRequestAllDriverRankingsResponse(
        fbBuffer
      );
    // Convert the flatbuffer list into an array. That's more useful.
    const driverRankingList = [];
    for (let i = 0; i < parsedResponse.driverRankingListLength(); i++) {
      driverRankingList.push(parsedResponse.driverRankingList(i));
    }
    return driverRankingList;
  }
  // Returns all data scouting entries from the database.
  async fetchStats2024List(): Promise<Stats2024[]> {
    let fbBuffer = await this.fetchFromServer(
      Request2024DataScouting.startRequest2024DataScouting,
      Request2024DataScouting.endRequest2024DataScouting,
      '/requests/request/2024_data_scouting'
    );

    const parsedResponse =
      Request2024DataScoutingResponse.getRootAsRequest2024DataScoutingResponse(
        fbBuffer
      );

    // Convert the flatbuffer list into an array. That's more useful.
    const statList = [];
    for (let i = 0; i < parsedResponse.statsListLength(); i++) {
      statList.push(parsedResponse.statsList(i));
    }
    return statList;
  }

  // Returns all pit image entries from the database.
  async fetchPitImagesList(): Promise<PitImage[]> {
    let fbBuffer = await this.fetchFromServer(
      RequestAllPitImages.startRequestAllPitImages,
      RequestAllPitImages.endRequestAllPitImages,
      '/requests/request/all_pit_images'
    );

    const parsedResponse =
      RequestAllPitImagesResponse.getRootAsRequestAllPitImagesResponse(
        fbBuffer
      );

    // Convert the flatbuffer list into an array. That's more useful.
    const pitImageList = [];
    for (let i = 0; i < parsedResponse.pitImageListLength(); i++) {
      pitImageList.push(parsedResponse.pitImageList(i));
    }
    return pitImageList;
  }
}
