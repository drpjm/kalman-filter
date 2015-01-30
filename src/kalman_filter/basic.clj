(ns kalman-filter.basic
  (:require [clojure.core.matrix :as mat]))
(mat/set-current-implementation :vectorz)

; Ok, basic idea(s): use noisy measurements to estimate the "state" of our system.
; The algorithm is recursive. Make a prediction of the future, get current sensor data,
; compare these values, adjust estimate based on the results. (Sounds like basic feedback to me!)

; Notation:
; x - state; u - input; P - system error estimate matrix; Q - process noise; R - measurement noise.
; z - current values to compute the next state.

; This code will follow the Linear Kalman Filter from http://greg.czerniak.info/guides/kalman1/

(defn linear-kalman-filter [A B H x-init P Q R]
  {:pre [(and (mat/matrix? A) (mat/matrix? B)
              (mat/matrix? H) (mat/matrix? x-init)
              (mat/matrix? P) (mat/matrix? Q) (mat/matrix? R))]}
  "An implementation of the linear Kalman filter for any n-dimensional system."
  (mat/pm A))