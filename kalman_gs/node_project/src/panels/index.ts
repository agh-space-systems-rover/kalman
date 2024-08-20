import Arms from './arm';
import Feeds from './feeds';
import Imu from './imu';
import Map from './map';
import MapHeader from './map.header';
import Supervisor from './supervisor';
import Waypoints from './waypoints';
import Wheels from './wheels';
import { IconDefinition } from '@fortawesome/fontawesome-svg-core';
import {
  faBullseye,
  faCube,
  faLocationDot,
  faVideo,
  faWheelchair,
  faDiagramProject,
  faFlag
} from '@fortawesome/free-solid-svg-icons';

// Add new panels here:
export type PanelID =
  | 'imu'
  | 'wheels'
  | 'supervisor'
  | 'map'
  | 'feeds'
  | 'arm'
  | 'waypoints';
export const defaultPanel: PanelID = 'map';
export const panelInfos: Panels = {
  imu: {
    Component: Imu,
    name: 'IMU',
    icon: faCube
  },
  wheels: {
    Component: Wheels,
    name: 'Wheels',
    icon: faWheelchair
  },
  supervisor: {
    Component: Supervisor,
    name: 'Supervisor',
    icon: faBullseye
  },
  map: {
    Component: Map,
    HeaderComponent: MapHeader,
    name: 'Map',
    icon: faLocationDot
  },
  feeds: {
    Component: Feeds,
    name: 'Feeds',
    icon: faVideo
  },
  arm: {
    Component: Arms,
    name: 'Arm',
    icon: faDiagramProject
  },
  waypoints: {
    Component: Waypoints,
    name: 'Waypoints',
    icon: faFlag
  }
};

// type definitions for the panels above
export type Panels = {
  [id in PanelID]: PanelInfo;
};
export type PanelInfo = {
  Component: React.ComponentType<any>;
  HeaderComponent?: React.ComponentType<any>;
  name: string;
  icon: IconDefinition;
};
