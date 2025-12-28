'use strict';

const path = require('path');
const webpack = require('webpack');
const HtmlWebpackPlugin = require('html-webpack-plugin');
const ProgressBarPlugin = require('progress-bar-webpack-plugin');
const CopyWebpackPlugin = require('copy-webpack-plugin');
const ESLintPlugin = require('eslint-webpack-plugin');
const ReactRefreshWebpackPlugin = require('@pmmmwh/react-refresh-webpack-plugin');
const FaviconsWebpackPlugin = require('favicons-webpack-plugin');

module.exports = (env, argv) => {
  const { mode } = argv;
  const isEnvDevelopment = mode === 'development';
  const isEnvProduction = mode === 'production';

  const cssRegex = /\.css$/;
  const sassRegex = /\.(scss|sass)$/;
  const lessRegex = /\.less$/;

  return {
    context: path.join(__dirname, 'src'),

    entry: {
      app: './app.js',
      parameters: path.join(__dirname, 'src/store/config/parameters.yml'),
    },

    output: {
      path: path.join(__dirname, 'dist'),
      filename: './[name].bundle.js',
      publicPath: '',
    },

    devtool: isEnvDevelopment ? 'inline-source-map' : 'hidden-source-map',

    resolve: {
      extensions: ['.jsx', '.js', '.json', '.scss', '.css', '.png', '.svg'],
      alias: {
        store: path.resolve(__dirname, 'src/store'),
        styles: path.resolve(__dirname, 'src/styles'),
        components: path.resolve(__dirname, 'src/components'),
        utils: path.resolve(__dirname, 'src/utils'),
        renderer: path.resolve(__dirname, 'src/renderer'),
        assets: path.resolve(__dirname, 'assets'),
        proto_bundle: path.resolve(__dirname, 'proto_bundle'),
        'three': path.resolve(__dirname, 'node_modules/three'),
        'three/examples/jsm': path.resolve(__dirname, 'node_modules/three/examples/jsm'),
        'three/addons': path.resolve(__dirname, 'node_modules/three/examples/jsm'),
      },
    },

    module: {
      rules: [
        {
          test: /\.(js|jsx)$/,
          exclude: /node_modules/,
          use: [
            {
              loader: 'babel-loader',
              options: {
                presets: [
                  ['@babel/preset-env', { targets: "defaults" }],
                  // 【核心修复2】解决 React is not defined 错误
                  ['@babel/preset-react', {
                    "runtime": "automatic"
                  }]
                ],
                plugins: [
                  isEnvDevelopment && require.resolve('react-refresh/babel'),
                  // MobX 6 推荐配置：装饰器在前，Class Properties 在后
                  ['@babel/plugin-proposal-decorators', { legacy: true }],
                  ['@babel/plugin-proposal-class-properties', { loose: true }],
                  '@babel/plugin-proposal-optional-chaining',
                  [
                    '@babel/plugin-transform-modules-commonjs',
                    { allowTopLevelThis: true },
                  ],
                  [
                    'import',
                    {
                      libraryName: 'antd',
                      libraryDirectory: 'es',
                      style: true,
                    },
                    'antd',
                  ],
                ].filter(Boolean),
              },
            },
          ],
        },
        {
          test: /\.mtl$|\.obj$/,
          exclude: /node_modules/,
          use: [{ loader: 'file-loader' }],
        },
        {
          test: /\.(png|jpe?g|svg|mp4|mov|gif)$/i,
          type: 'asset',
          generator: {
            filename: 'assets/[hash][ext][query]',
          },
        },
        {
          test: cssRegex,
          use: [{ loader: 'style-loader' }, { loader: 'css-loader' }]
        },
        {
          test: sassRegex,
          use: [
            { loader: 'style-loader' },
            {
              loader: 'css-loader',
              options: { modules: { mode: 'global' } }
            },
            {
              loader: 'sass-loader',
              options: {
                sassOptions: { includePaths: ['./node_modules'] },
              },
            },
          ],
        },
        {
          test: lessRegex,
          use: [
            { loader: 'style-loader' },
            { loader: 'css-loader' },
            {
              loader: 'less-loader',
              options: { lessOptions: { javascriptEnabled: true } }
            },
          ],
        },
        {
          test: /\.woff(2)?(\?v=[0-9]\.[0-9]\.[0-9])?$/,
          use: [
            {
              loader: 'url-loader',
              options: {
                limit: 10000,
                mimetype: 'application/font-woff',
              },
            },
          ],
        },
        {
          test: /\.(ttf|eot|svg)(\?v=[0-9]\.[0-9]\.[0-9])?$/,
          loader: 'file-loader',
        },
        {
          test: /webworker\.js$/,
          use: [
            {
              loader: 'worker-loader',
              options: { filename: 'worker.bundle.js' },
            },
            {
              loader: 'babel-loader',
              options: { presets: ['@babel/preset-env'] },
            }],
        },
        {
          oneOf: [
            {
              test: /parameters.yml/,
              use: [
                {
                  loader: 'file-loader',
                  options: {
                    name: '[path][name].json',
                    context: 'src/store/config',
                    outputPath: '.',
                  },
                },
                {
                  loader: 'yaml-loader',
                  options: { asJSON: true }
                },
              ]
            },
            {
              test: /\.yml$/,
              exclude: /node_modules/,
              use: [
                'json-loader',
                {
                  loader: 'yaml-loader',
                  options: { asJSON: true }
                },
              ],
            },
          ]
        }
      ],
    },

    plugins: [
      new ProgressBarPlugin({
        format: 'build [:bar] :percent (:elapsed seconds)',
        clear: false,
      }),
      isEnvDevelopment && new ReactRefreshWebpackPlugin(),
      new ESLintPlugin({
        fix: true,
        lintDirtyModulesOnly: true,
        extensions: ['js', 'jsx'],
      }),
      new HtmlWebpackPlugin({
        template: './index.hbs',
        chunks: ['app'],
      }),
      new FaviconsWebpackPlugin({
        logo: './favicon.png',
        cache: true,
        prefix: 'icons/',
      }),
      new CopyWebpackPlugin([
        {
          from: '../node_modules/three/examples/fonts',
          to: 'fonts',
        },
      ]),
      new webpack.IgnorePlugin({
        resourceRegExp: /^\.\/locale$/,
        contextRegExp: /moment$/,
      }),
      new webpack.DefinePlugin({
        OFFLINE_PLAYBACK: JSON.stringify(false),
      }),
    ].filter(Boolean),

    devServer: {
      client: { progress: true },
      static: {
        directory: path.join(__dirname, 'src'),
        watch: true,
      },
      hot: true,
      open: true,
      compress: true,
      port: 8080,
    },

    performance: { hints: false },
  };
};
