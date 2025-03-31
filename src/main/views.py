from django.http import JsonResponse
from django.shortcuts import render
from pathlib import Path
import json

def map_view(request):
    return render(request, 'main/map.html')

def marker_data(request):
    path = Path('marker_data.json')
    if path.exists():
        with path.open() as f:
            data = json.load(f)
        return JsonResponse(data)
    # 마커 없을 때는 빈 리스트
    return JsonResponse({'markers': []})