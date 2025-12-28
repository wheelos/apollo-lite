import classNames from 'classnames';
import { inject, observer } from 'mobx-react';
import React from 'react';

import { millisecondsToTime } from 'utils/misc';

// 保持为 PureComponent 以优化性能，只有 props 改变时才重绘
class Delay extends React.PureComponent {
  render() {
    const { time, warning } = this.props;

    // 使用位运算符 | 0 进行取整，如果 time 是 '-' 则保持原样
    const timeString = (time === '-') ? time : millisecondsToTime(time | 0);

    return (
      <div className={classNames({ value: true, warning })}>
        {timeString}
      </div>
    );
  }
}

@inject('store') @observer
export default class DelayTable extends React.Component {
  render() {
    const { moduleDelay } = this.props.store;

    const sortedKeys = Array.from(moduleDelay.keys()).sort();

    const items = sortedKeys.map((key) => {
      const module = moduleDelay.get(key);

      // 这里的逻辑：如果延迟超过 2s 且不是红绿灯模块，则显示警告颜色
      const delayValue = parseFloat(module.delay);
      const isNumericDelay = !isNaN(delayValue);
      const warning = isNumericDelay && delayValue > 2000 && module.name !== 'TrafficLight';

      return (
        <div className="delay-item" key={`delay_${key}`}>
          <div className="name">{module.name}</div>
          <Delay time={module.delay} warning={warning} />
        </div>
      );
    });

    return (
      <div className="delay card">
        <div className="card-header"><span>Module Delay</span></div>
        <div className="card-content-column">
          {items.length > 0 ? items : <div className="no-data">No module delay data</div>}
        </div>
      </div>
    );
  }
}
