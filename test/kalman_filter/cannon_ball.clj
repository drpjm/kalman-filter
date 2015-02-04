(ns kalman-filter.cannon-ball
  (:require [kalman-filter.basic :as lkf]
            [clojure.core.matrix :as mat]
            [incanter.core :as incanter]
            [incanter.stats :as stats]
            [incanter.charts :as c]))

; To create a linear Kalman filter, you invoke the linear-kalman-filter function, which
; returns a function of four variables: state, covariance, measurement, and input.

; Here is the cannonball sensor measurement example. The "sensor" fn keeps track
; of the actual location (using physics eqns and a time) and outputs 
; "noisy" sensor data that is in the relative range of the ball,
; in a measurement matrix (this is the z vec for filter)
(def sensor-data (atom []))

(defn camera-data [t]
  "Takes n samples from a noisy camera sensor."
  (let [x-vel  (* 100 (Math/cos (/ 3.14 4)))
        x-pos  (* t x-vel)
        y-vel  (- (* 100 (Math/sin (/ 3.14 4))) (* 9.81 t))
        y-pos  (- (* t (* 100 (Math/sin (/ 3.14 4)))) (* 0.5 9.81 (* t t)))
        ;add gaussian noise
        x-vel  (+ (- 2.5 (rand 5)) x-vel) ;(stats/sample-normal 1 :mean x-vel :std 0.1)
        x-pos  (+ (- 50 (rand 100)) x-pos);(stats/sample-normal 1 :mean x-pos :std 0.1)
        y-vel  (+ (- 2.5 (rand 5)) y-vel);(stats/sample-normal 1 :mean y-vel :std 0.1)
        y-pos  (+ (- 50 (rand 100)) y-pos)];(stats/sample-normal 1 :mean y-pos :std 0.1)]
    (reset! sensor-data (conj @sensor-data {:x x-pos :y y-pos}))
    (mat/matrix [[x-pos]
                 [x-vel]
                 [y-pos]
                 [y-vel]])))

(defn- true-path []
  "Returns a seq of vals that represent the true x and y coords
   over time"
  (loop [idx 0
         locs []]
    (let[t  (* 0.1 idx) 
         x-vel  (* 100 (Math/cos (/ 3.14 4)))
         x-pos  (* t x-vel)
         y-pos  (- (* t (* 100 (Math/sin (/ 3.14 4)))) (* 0.5 9.81 (* t t)))]
      (if (>= idx 150) 
        locs
        (recur 
          (inc idx)
          (conj locs {:true-x x-pos :true-y y-pos}))))))
    
    

(let [delta-t 0.1
      
      g 9.81
      
      A (mat/matrix [[1 delta-t  0 0]
                     [0 1        0 0]
                     [0 0        1 delta-t]
                     [0 0        0 1]])
      B (mat/matrix [[0 0 0 0]
                     [0 0 0 0]
                     [0 0 1 0]
                     [0 0 0 1]])
      H (mat/matrix [[1 0 0 0]
                     [0 1 0 0]
                     [0 0 1 0]
                     [0 0 0 1]])
      Q (mat/matrix [[0 0 0 0]
                     [0 0 0 0]
                     [0 0 0 0]
                     [0 0 0 0]])
      R (mat/matrix [[0.1 0   0   0  ]
                     [0   0.1 0   0  ]
                     [0   0   0.1 0  ]
                     [0   0   0   0.1]])
      loc-update-fn (lkf/linear-kalman-filter A B H Q R)]
  ; Store the results for plotting.
  (let [steps 150
        estimated-states (:state (loop [idx 0
                                        curr-results (loc-update-fn 
                                                       (mat/matrix [[0]
                                                                    [(* 100 (Math/cos (/ 3.14 4)))]
                                                                    [500]
                                                                    [(* 100 (Math/sin (/ 3.14 4)))]]) 
                                                       (mat/matrix [[1 0 0 0]
                                                                    [0 1 0 0]
                                                                    [0 0 1 0]
                                                                    [0 0 0 1]]) 
                                                       (mat/matrix [[0]
                                                                    [0]
                                                                    [(* -0.5 g (* delta-t delta-t))]
                                                                    [(* -1 g delta-t)]])  
                                                       (camera-data (* delta-t idx)))
                                        state-history []]
                                  (if (>= idx steps)
                                    {:state state-history}
                                    (recur (inc idx) 
                                           (loc-update-fn (:state curr-results) 
                                                          (:covar-matrix curr-results) 
                                                          (mat/matrix [[0]
                                                                [0]
                                                                [(* -0.5 g (* delta-t delta-t))]
                                                                [(* -1 g delta-t)]]) 
                                                          (camera-data (* delta-t idx)))
                                           ; State is returned within a Matrix data structure
                                           (conj state-history {:x-loc (mat/mget (:state curr-results) 0 0)
                                                                :y-loc (mat/mget (:state curr-results) 2 0)})))))
        x-locs (map :x-loc estimated-states)
        y-locs (map :y-loc estimated-states)
        true-locs (true-path)
        true-x-locs (map :true-x true-locs)
        true-y-locs (map :true-y true-locs)
        
        camera-x-locs (map :x @sensor-data)
        camera-y-locs (map :y @sensor-data)
        
        
        cannon-plot (c/xy-plot x-locs y-locs
                               :title "Cannonball Location"
                               :x-label "X position"
                               :y-label "Y-position")]
    
    (c/add-lines cannon-plot true-x-locs true-y-locs)
    (c/add-lines cannon-plot camera-x-locs camera-y-locs)
    (incanter/view cannon-plot)))
