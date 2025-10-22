import pandas as pd
import matplotlib.pyplot as plt

# --- Load and preprocess ---
df = pd.read_csv("Pilot_study.csv")
df.columns = df.columns.str.strip().str.replace('"', '').str.replace("'", '')

mapping = {
    "5 – Strongly Agree": 5,
    "4 – Agree": 4,
    "3 – Neutral": 3,
    "2 – Disagree": 2,
    "1 – Strongly Disagree": 1
}
likert_cols = [c for c in df.columns if "Please rate" in c]
for col in likert_cols:
    df[col] = df[col].map(mapping)

# --- Compute mean per participant ---
df["MeanScore"] = df[likert_cols].mean(axis=1)

# --- Detect participants with slight (occasional) experience ---
df["SlightExperience"] = df["Have you used VR or MR headsets before?"].str.contains("Occasionally", case=False, na=False)

# --- Plot setup ---
fig, ax = plt.subplots(figsize=(8,5))
participants = [f"P{i+1}" for i in range(len(df))]

# --- Color highlighting ---
colors = ["#6A5ACD" if not used else "#B39DDB" for used in df["SlightExperience"]]  # softer lavender tone

# --- Plot bars ---
bars = ax.bar(participants, df["MeanScore"], color=colors, edgecolor="black")

# Add value labels on top
for bar in bars:
    height = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2, height + 0.05, f"{height:.2f}",
            ha='center', va='bottom', fontsize=8)

# --- Add group mean line ---
overall_mean = df["MeanScore"].mean()
ax.axhline(overall_mean, color='red', linestyle='--', linewidth=1.5, label=f"Group Mean = {overall_mean:.2f}")

# --- Legend and styling ---
from matplotlib.patches import Patch
legend_elements = [
    Patch(facecolor="#B39DDB", edgecolor="black", label="Slight Experience"),
    Patch(facecolor="#6A5ACD", edgecolor="black", label="No Experience"),
    Patch(facecolor="none", edgecolor="none", label=f"Group Mean = {overall_mean:.2f}")
]
ax.legend(handles=legend_elements, loc='lower right', fontsize=9)

ax.set_ylim(0,5)
ax.set_xlabel("Participant", fontsize=10)
ax.set_ylabel("Average Rating", fontsize=10)
ax.set_title("Individual Participant Scores", fontsize=12, weight='bold')
ax.grid(axis='y', linestyle='--', alpha=0.5)

plt.tight_layout()
plt.savefig("PilotStudy_ParticipantScores_SlightExperience.png", dpi=300, bbox_inches='tight')
plt.savefig("PilotStudy_ParticipantScores_SlightExperience.pdf", bbox_inches='tight')
plt.show()
