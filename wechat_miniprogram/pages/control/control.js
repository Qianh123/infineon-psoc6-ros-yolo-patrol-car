const API_BASE_URL = 'http://172.20.10.4:8000'
const STATUS_URL = `${API_BASE_URL}/api/status`
const STATUS_REFRESH_INTERVAL = 1000
const FRAME_REFRESH_INTERVAL = 300

const API = {
  modeManual: `${API_BASE_URL}/api/mode/manual`,
  modeAuto: `${API_BASE_URL}/api/mode/auto`,
  manualMove: `${API_BASE_URL}/api/manual/move`,
  stop: `${API_BASE_URL}/api/stop`,
  resume: `${API_BASE_URL}/api/resume`
}

Page({
  data: {
    pageTitle: '仓库智能巡检小车控制台',
    baseUrl: API_BASE_URL,
    videoMode: 'mjpeg',
    mjpegUrl: 'http://172.20.10.4:8090/stream.mjpg',
    frameUrl: 'http://172.20.10.4:8090/frame.jpg',
    videoUrl: 'http://172.20.10.4:8090/stream.mjpg',
    videoModeText: 'MJPEG模式',
    statusUrl: STATUS_URL,
    currentMode: '',
    modeText: '等待状态',
    speedOptions: [30, 50, 70, 90],
    selectedSpeed: 70,
    videoStatus: '视频流加载中',
    statusMessage: '正在连接状态接口',
    lastUpdateTime: '等待刷新',
    lastActionText: '暂无控制操作',
    statusRows: [],
    isStatusOnline: false,
    modeLoading: '',
    manualBusy: false,
    manualLoadingDirection: '',
    stopLoading: false,
    resumeLoading: false
  },

  statusTimer: null,
  statusRequesting: false,
  videoTimer: null,
  frameLoading: false,

  onLoad() {
    wx.setNavigationBarTitle({
      title: this.data.pageTitle
    })

    this.applyVideoMode()

    this.updateStatusData({})
    this.refreshStatus()
    this.startStatusPolling()
  },

  onUnload() {
    this.clearStatusPolling()
    this.clearFramePolling()
    this.frameLoading = false
  },

  onPullDownRefresh() {
    this.refreshStatus(() => {
      wx.stopPullDownRefresh()
    })
  },

  startStatusPolling() {
    this.clearStatusPolling()
    this.statusTimer = setInterval(() => {
      this.refreshStatus()
    }, STATUS_REFRESH_INTERVAL)
  },

  clearStatusPolling() {
    if (this.statusTimer) {
      clearInterval(this.statusTimer)
      this.statusTimer = null
    }
  },

  switchVideoMode() {
    const nextMode = this.data.videoMode === 'mjpeg' ? 'frame' : 'mjpeg'

    this.setData({
      videoMode: nextMode
    })

    this.applyVideoMode(nextMode)
  },

  applyVideoMode(mode = this.data.videoMode) {
    if (mode === 'frame') {
      this.setData({
        videoModeText: '真机兼容模式'
      })
      this.startFramePolling()
      return
    }

    this.clearFramePolling()
    this.setData({
      videoUrl: this.data.mjpegUrl,
      videoModeText: 'MJPEG模式'
    })
  },

  startFramePolling() {
    this.clearFramePolling()
    this.loadFrameImage()
    this.videoTimer = setInterval(() => {
      this.loadFrameImage()
    }, FRAME_REFRESH_INTERVAL)
  },

  clearFramePolling() {
    if (this.videoTimer) {
      clearInterval(this.videoTimer)
      this.videoTimer = null
    }
  },

  loadFrameImage() {
    if (this.data.videoMode !== 'frame') {
      return
    }

    if (this.frameLoading) {
      return
    }

    this.frameLoading = true

    wx.request({
      url: this.data.frameUrl + '?t=' + Date.now(),
      method: 'GET',
      responseType: 'arraybuffer',
      timeout: 1000,
      success: (res) => {
        if (this.data.videoMode !== 'frame') {
          return
        }

        if (res.statusCode >= 200 && res.statusCode < 300 && res.data) {
          const base64 = wx.arrayBufferToBase64(res.data)
          this.setData({
            videoUrl: 'data:image/jpeg;base64,' + base64
          })
          return
        }

        console.error('frame request failed:', res)
      },
      fail: (err) => {
        console.error('frame request failed:', err)
      },
      complete: () => {
        this.frameLoading = false
      }
    })
  },

  refreshStatus(done) {
    if (this.statusRequesting) {
      if (typeof done === 'function') {
        done()
      }
      return
    }

    this.statusRequesting = true

    wx.request({
      url: STATUS_URL,
      method: 'GET',
      timeout: 10000,
      success: (res) => {
        if (res.statusCode === 200 && res.data) {
          const statusData = this.parseStatusData(res.data)
          this.updateStatusData(statusData)
          this.setData({
            isStatusOnline: true,
            statusMessage: '状态接口在线',
            lastUpdateTime: this.formatNow()
          })
          return
        }

        console.error('status request failed:', res)
        this.setData({
          isStatusOnline: false,
          statusMessage: '状态接口连接失败',
          lastUpdateTime: this.formatNow()
        })
      },
      fail: (err) => {
        console.error('status request failed:', err)
        this.setData({
          isStatusOnline: false,
          statusMessage: '状态接口连接失败',
          lastUpdateTime: this.formatNow()
        })
      },
      complete: () => {
        this.statusRequesting = false
        if (typeof done === 'function') {
          done()
        }
      }
    })
  },

  parseStatusData(data) {
    if (typeof data !== 'string') {
      return data
    }

    try {
      return JSON.parse(data)
    } catch (err) {
      console.error('status request failed:', {
        message: 'invalid JSON response',
        data,
        err
      })
      return {}
    }
  },

  updateStatusData(statusData) {
    const data = statusData && typeof statusData === 'object' ? statusData : {}
    const backendMode = this.normalizeMode(data.mode)
    const modeText = this.formatModeText(backendMode)
    const state = this.normalizeStatusValue(data.state)
    const mode = this.normalizeStatusValue(data.mode)
    const person = this.normalizeStatusValue(data.person)
    const cargo = this.normalizeStatusValue(data.cargo)
    const distance = this.normalizeStatusValue(data.distance)
    const reason = this.normalizeStatusValue(data.reason)
    const close = this.normalizeStatusValue(data.close)
    const slowDown = this.normalizeStatusValue(data.slow_down)
    const emergencyStop = this.normalizeStatusValue(data.emergency_stop)

    this.setData({
      currentMode: backendMode,
      modeText,
      statusRows: [
        {
          key: 'state',
          label: '当前状态 state',
          value: this.formatText(state),
          valueClass: 'value-strong'
        },
        {
          key: 'mode',
          label: '当前模式 mode',
          value: this.formatModeStatus(mode, backendMode),
          valueClass: 'value-strong'
        },
        {
          key: 'person',
          label: '人数 person',
          value: this.formatText(person)
        },
        {
          key: 'cargo',
          label: '货物数量 cargo',
          value: this.formatText(cargo)
        },
        {
          key: 'distance',
          label: '超声波距离 distance',
          value: this.formatDistance(distance),
          valueClass: 'value-strong'
        },
        {
          key: 'reason',
          label: '触发原因 reason',
          value: this.formatText(reason),
          wide: true
        },
        {
          key: 'close',
          label: '接近障碍 close',
          value: this.formatBoolean(close),
          valueClass: close === true ? 'value-warning' : ''
        },
        {
          key: 'slow_down',
          label: '减速 slow_down',
          value: this.formatBoolean(slowDown),
          valueClass: slowDown === true ? 'value-warning' : ''
        },
        {
          key: 'emergency_stop',
          label: '急停 emergency_stop',
          value: this.formatBoolean(emergencyStop),
          valueClass: emergencyStop === true ? 'value-danger' : ''
        }
      ]
    })
  },

  switchMode(e) {
    const targetMode = e.currentTarget.dataset.mode
    const normalizedMode = this.normalizeMode(targetMode)

    if (!normalizedMode || this.data.modeLoading) {
      return
    }

    const url = normalizedMode === 'manual' ? API.modeManual : API.modeAuto
    const label = this.formatModeText(normalizedMode)

    this.setData({
      modeLoading: normalizedMode,
      lastActionText: `切换到${label}中`
    })

    wx.request({
      url,
      method: 'POST',
      timeout: 5000,
      success: (res) => {
        const data = this.parseStatusData(res.data)

        if (data && data.ok === true) {
          this.setData({
            currentMode: normalizedMode,
            modeText: label,
            lastActionText: `已切换到${label} ${this.formatNow()}`
          })
          this.refreshStatus()
          return
        }

        console.error('mode switch request failed:', res)
        this.setData({
          lastActionText: `切换到${label}失败`
        })
        wx.showToast({
          title: '模式切换失败',
          icon: 'none'
        })
      },
      fail: (err) => {
        console.error('mode switch request failed:', err)
        this.setData({
          lastActionText: `切换到${label}失败`
        })
        wx.showToast({
          title: '模式切换失败',
          icon: 'none'
        })
      },
      complete: () => {
        this.setData({
          modeLoading: ''
        })
      }
    })
  },

  selectSpeed(e) {
    const speed = Number(e.currentTarget.dataset.speed)

    if (this.data.speedOptions.indexOf(speed) === -1) {
      return
    }

    this.setData({
      selectedSpeed: speed,
      lastActionText: `速度已选择 ${speed}`
    })
  },

  onMoveTap(e) {
    const direction = e.currentTarget.dataset.direction
    const validDirections = ['forward', 'back', 'left', 'right', 'stop']

    if (validDirections.indexOf(direction) === -1 || this.data.manualBusy) {
      return
    }

    if (this.data.currentMode !== 'manual') {
      wx.showToast({
        title: '请先切换到手动控制',
        icon: 'none'
      })
      return
    }

    this.setData({
      manualBusy: true,
      manualLoadingDirection: direction,
      lastActionText: `发送${this.formatDirectionText(direction)}指令中`
    })

    wx.request({
      url: API.manualMove,
      method: 'POST',
      timeout: 5000,
      header: {
        'content-type': 'application/json'
      },
      data: {
        direction,
        speed: this.data.selectedSpeed
      },
      success: (res) => {
        const data = this.parseStatusData(res.data)

        if (res.statusCode === 409 || (data && data.ok === false)) {
          console.error('manual move request failed:', res)
          this.setData({
            lastActionText: `${this.formatDirectionText(direction)}失败`
          })
          wx.showToast({
            title: '请先切换到手动控制',
            icon: 'none'
          })
          return
        }

        if (this.isOkResponse(res)) {
          this.setData({
            lastActionText: `${this.formatDirectionText(direction)}成功 ${this.formatNow()}`
          })
          this.refreshStatus()
          return
        }

        console.error('manual move request failed:', res)
        this.setData({
          lastActionText: `${this.formatDirectionText(direction)}失败`
        })
        wx.showToast({
          title: '手动控制失败',
          icon: 'none'
        })
      },
      fail: (err) => {
        console.error('manual move request failed:', err)
        this.setData({
          lastActionText: `${this.formatDirectionText(direction)}失败`
        })
        wx.showToast({
          title: '手动控制失败',
          icon: 'none'
        })
      },
      complete: () => {
        this.setData({
          manualBusy: false,
          manualLoadingDirection: ''
        })
      }
    })
  },

  onStopTap() {
    this.requestControl(API.stop, '急停', 'stopLoading')
  },

  onResumeTap() {
    this.requestControl(API.resume, '恢复巡检', 'resumeLoading')
  },

  requestControl(url, label, loadingKey) {
    if (this.data[loadingKey]) {
      return
    }

    this.setData({
      [loadingKey]: true,
      lastActionText: `${label}请求发送中`
    })

    wx.request({
      url,
      method: 'POST',
      timeout: 5000,
      success: (res) => {
        if (this.isOkResponse(res)) {
          this.setData({
            lastActionText: `${label}成功 ${this.formatNow()}`
          })
          this.refreshStatus()
          return
        }

        console.error(`${label} request failed:`, res)
        this.setData({
          lastActionText: `${label}失败`
        })
        wx.showToast({
          title: `${label}失败`,
          icon: 'none'
        })
      },
      fail: (err) => {
        console.error(`${label} request failed:`, err)
        this.setData({
          lastActionText: `${label}请求失败`
        })
        wx.showToast({
          title: `${label}请求失败`,
          icon: 'none'
        })
      },
      complete: () => {
        this.setData({
          [loadingKey]: false
        })
      }
    })
  },

  isOkResponse(res) {
    if (!res || res.statusCode < 200 || res.statusCode >= 300) {
      return false
    }

    if (res.data === undefined || res.data === null || res.data === '') {
      return true
    }

    const data = this.parseStatusData(res.data)
    return !data || typeof data !== 'object' || data.ok !== false
  },

  normalizeMode(mode) {
    if (mode === 'manual' || mode === 'auto') {
      return mode
    }

    return ''
  },

  normalizeStatusValue(value) {
    return value === undefined ? null : value
  },

  formatModeText(mode) {
    if (mode === 'manual') {
      return '手动控制'
    }

    if (mode === 'auto') {
      return 'YOLO自主巡检'
    }

    return '等待状态'
  },

  formatModeStatus(value, fallbackMode) {
    const mode = this.normalizeMode(value)

    if (mode) {
      return `${mode} / ${this.formatModeText(mode)}`
    }

    if (value === undefined || value === null || value === '') {
      return fallbackMode ? `${fallbackMode} / ${this.formatModeText(fallbackMode)}` : '--'
    }

    return String(value)
  },

  formatDirectionText(direction) {
    const directionMap = {
      forward: '前进',
      back: '后退',
      left: '左转',
      right: '右转',
      stop: '停止'
    }

    return directionMap[direction] || direction
  },

  formatText(value) {
    if (value === undefined || value === null || value === '') {
      return '--'
    }

    return String(value)
  },

  formatBoolean(value) {
    if (value === true) {
      return '是'
    }

    if (value === false) {
      return '否'
    }

    return '未知'
  },

  formatDistance(value) {
    if (value === null || value === undefined || value === '') {
      return '无效'
    }

    const distance = Number(value)
    if (!Number.isFinite(distance)) {
      return '无效'
    }

    return `${distance.toFixed(1)} cm`
  },

  onVideoLoad() {
    this.setData({
      videoStatus: '视频流已加载'
    })
  },

  onVideoError(err) {
    console.error('video stream load failed:', err)
    this.setData({
      videoStatus: '视频流加载失败'
    })
  },

  formatNow() {
    const now = new Date()
    const pad = (value) => {
      const text = String(value)
      return text.length < 2 ? `0${text}` : text
    }

    return `${pad(now.getHours())}:${pad(now.getMinutes())}:${pad(now.getSeconds())}`
  }
})
