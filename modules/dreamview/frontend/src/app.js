import { Provider } from 'mobx-react';
import { createRoot } from 'react-dom/client';

import 'styles/antd-reset.scss';
import 'styles/main.scss';

import Dreamview from 'components/Dreamview';
import STORE from 'store';

// 1. 获取挂载点
const container = document.getElementById('root');

// 2. 创建 React 根节点 (开启 React 18 的并发特性)
const root = createRoot(container);

// 3. 渲染应用
root.render(
    <Provider store={STORE}>
        <Dreamview />
    </Provider>,
);
