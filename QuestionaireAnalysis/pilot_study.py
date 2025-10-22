import pandas as pd
import matplotlib.pyplot as plt

# --- Load CSV ---
df = pd.read_csv("Pilot_study.csv")

# --- Clean headers: remove extra spaces, quotes, and invisible characters ---
df.columns = df.columns.str.strip().str.replace('"', '').str.replace("'", '')

# Debug: print headers to confirm
print("Cleaned columns:\n", list(df.columns))

# --- Fix Likert text → numeric mapping ---
mapping = {
    "5 – Strongly Agree": 5,
    "4 – Agree": 4,
    "3 – Neutral": 3,
    "2 – Disagree": 2,
    "1 – Strongly Disagree": 1
}

# Identify all Likert questions
likert_cols = [c for c in df.columns if "Please rate" in c]
for col in likert_cols:
    df[col] = df[col].map(mapping)

# --- Graph 2: Improved Thesis-Ready Likert Averages ---
import numpy as np

means = df[likert_cols].mean().sort_values(ascending=True)

# Shorten labels for clarity
labels = [
    "Drone awareness",
    "Swarm state understanding",
    "Identify tasks per drone",
    "Spatial confidence",
    "Interaction felt natural",
    "Controller precision",
    "Layout clarity",
    "Ease of learning",
    "Waypoint definition",
    "Modify missions easily",
    "Planning efficiency",
    "System stability",
    "Feedback clarity",
    "Confidence in real ops",
    "Overall MR Interaction"
]

# Ensure labels match order of sorted means
if len(labels) == len(means):
    means.index = labels
else:
    print("Warning: label count mismatch; using defaults.")

fig, ax = plt.subplots(figsize=(8,6))
bars = ax.barh(means.index, means.values, color="#6A5ACD", edgecolor="black", height=0.55)

# --- Add global average line ---
overall_mean = means.mean()
ax.axvline(overall_mean, color='red', linestyle='--', linewidth=1.5, label=f"Average = {overall_mean:.2f}")
ax.legend(loc='lower right', fontsize=9)

# Annotate bars with exact mean values
for bar in bars:
    width = bar.get_width()
    ax.text(width + 0.05, bar.get_y() + bar.get_height()/2,
            f"{width:.2f}", va='center', fontsize=9)

# Aesthetic improvements
ax.set_xlim(0,5)
ax.set_xlabel("Average Rating (1 = Strongly Disagree, 5 = Strongly Agree)", fontsize=10)
ax.set_title("Participant Evaluation of the MR Interface", fontsize=12, weight='bold')
ax.grid(axis='x', linestyle='--', alpha=0.5)

plt.tight_layout()

# 🔹 Save figure (both vector PDF and high-res PNG)
plt.savefig("PilotStudy_MR_Interface_Ratings.pdf", bbox_inches='tight')
plt.savefig("PilotStudy_MR_Interface_Ratings.png", dpi=300, bbox_inches='tight')

plt.show()
