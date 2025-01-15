import {NgModule, isDevMode} from '@angular/core';
import {BrowserModule} from '@angular/platform-browser';
import {BrowserAnimationsModule} from '@angular/platform-browser/animations';
import {ServiceWorkerModule} from '@angular/service-worker';

import {App} from './app';
import {PipeModule} from '@aos/scouting/www/pipes';
import {EntryModule} from '@aos/scouting/www/entry';
import {MatchListModule} from '@aos/scouting/www/match_list';
import {NotesModule} from '@aos/scouting/www/notes';
import {ShiftScheduleModule} from '@aos/scouting/www/shift_schedule';
import {ViewModule} from '@aos/scouting/www/view';
import {DriverRankingModule} from '@aos/scouting/www/driver_ranking';
import {PitScoutingModule} from '@aos/scouting/www/pit_scouting';
import {ScanModule} from '@aos/scouting/www/scan';

@NgModule({
  declarations: [App],
  imports: [
    BrowserModule,
    BrowserAnimationsModule,
    ServiceWorkerModule.register('./ngsw-worker.js', {
      enabled: !isDevMode(),
      // Register the ServiceWorker as soon as the application is stable
      // or after 30 seconds (whichever comes first).
      registrationStrategy: 'registerWhenStable:30000',
    }),
    EntryModule,
    NotesModule,
    MatchListModule,
    PipeModule,
    ShiftScheduleModule,
    DriverRankingModule,
    ViewModule,
    PitScoutingModule,
    ScanModule,
  ],
  exports: [App],
  bootstrap: [App],
})
export class AppModule {}
