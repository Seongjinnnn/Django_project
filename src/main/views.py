from django.http import JsonResponse
from django.shortcuts import render
from pathlib import Path
import json

def map_view(request):
    return render(request, 'main/map.html')

def vehicle_info(request):
    path = Path('vehicle_info.json')
    if path.exists():
        with path.open() as f:
            data = json.load(f)
        return JsonResponse(data)
    
    return JsonResponse({'vehicle': []})