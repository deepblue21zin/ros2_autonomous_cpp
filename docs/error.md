# Error Log

빌드 및 실행 중 발생한 에러와 해결 방법을 기록합니다.

---

## 2026-01-29

### Error 1: CMake Cache Conflict

**에러 메시지:**
```
CMake Error: The current CMakeCache.txt directory /root/ros2_ws/build/arduino_driver/CMakeCache.txt
is different than the directory /home/deepblue/ros2_autonomous_cpp/build/arduino_driver where
CMakeCache.txt was created.
```

**원인:**
- 호스트에서 빌드한 캐시가 Docker 컨테이너 내부 경로와 충돌
- `build/`, `install/`, `log/` 폴더가 다른 환경에서 생성됨

**해결:**
```bash
# 컨테이너 내부에서 빌드 캐시 삭제
docker exec adas_container bash -c "cd /root/ros2_ws && rm -rf build/ install/ log/"

# 다시 빌드
docker exec adas_container bash -c "source /opt/ros/humble/setup.bash && cd /root/ros2_ws && colcon build --symlink-install"
```

---

### Error 2: compose.yaml Indentation Error

**에러 메시지:**
```
services must be a mapping
```

**원인:**
- YAML 파일의 `services:` 키가 들여쓰기되어 있었음

**해결:**
- `services:`를 파일 맨 앞(들여쓰기 없이)으로 수정

---

### Error 3: GitHub Large File Error (820MB)

**에러 메시지:**
```
remote: error: File .vscode/browse.vc.db is 819.95 MB;
this exceeds GitHub's file size limit of 100.00 MB
```

**원인:**
- VSCode의 IntelliSense 캐시 파일이 커밋에 포함됨

**해결:**
```bash
# 커밋 취소
git reset --soft HEAD~1

# .vscode 폴더 스테이징 해제
git restore --staged .vscode/

# .gitignore에 추가 (이미 있음)
# .vscode/
# *.db

# 다시 커밋 & 푸시
git commit -m "message"
git push origin main
```

---

## Warning (무시 가능)

### perception_pkg 빌드 경고

```cpp
warning: unused parameter 'ys' [-Wunused-parameter]
warning: unused variable 'w' [-Wunused-variable]
warning: unused parameter 'roi_y' [-Wunused-parameter]
```

**파일:** `src/perception_pkg/src/lane_marking_node.cpp`

**상태:** 기능에 영향 없음, 추후 정리 예정
