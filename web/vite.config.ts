// vite.config.js
import { resolve } from 'path'
import { defineConfig } from 'vite'

// Tree shaking doesn't seem to work with plotly.js and it increases build times,
// so let's stick with plotly.js-dist-min.

export default defineConfig({
  build: {
    rollupOptions: {
      input: {
        deadWheelAngularRamp: resolve(__dirname, 'tuning/dead-wheel-angular-ramp.html'),
        driveEncoderAngularRamp: resolve(__dirname, 'tuning/drive-encoder-angular-ramp.html'),
        forwardRamp: resolve(__dirname, 'tuning/forward-ramp.html'),
        lateralRamp: resolve(__dirname, 'tuning/lateral-ramp.html'),
      },
    },
  },
})
